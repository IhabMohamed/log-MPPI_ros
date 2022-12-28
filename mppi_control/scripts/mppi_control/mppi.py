"""
@authors: Grady Williams, Autonomous Control and Decision Systems Laboratory, Georgia Institute of Technology, USA
          Ihab S. Mohamed, Vehicle Autonomy and Intelligence Lab, Indiana University, Bloomington, USA        
"""
"""
@brief: The implementation of the MPPI control strategy as proposed by Williams in "Model predictive
        path integral control: From theory to parallel computation", as well as its extension to log-MPPI. 
"""
import numpy as np
from scipy import interpolate
import scipy.signal

from jinja2 import Template

import warnings
import sys

import rospy

import pycuda.driver as cuda
import pycuda.autoinit
from pycuda.compiler import SourceModule

# Import pyCuda modules for computations
from pycuda.curandom import XORWOWRandomNumberGenerator
from pycuda import gpuarray

class MPPI_Controller:
    """
    Model predictive path integral controller. Computes an approximation of an optimal open loop
    control sequence by sampling and evaluation trajectories from the system dynamics. Sampling
    is performed in parallel on a GPU using the pycuda module, a python interface to Nvidia's CUDA
    architecture, this Requires an Nvidia GPU. The costs, dynamics, and (optionally) an initial
    policy to sample around are given as arguments to the constructor.

    Methods:
    default_policy -- The default control policy (All zeros) which the controller samples around.
    default_initialization_policy -- Policy for initializing new controls.
    initialize_controls -- Allow the controller iterate many times when starting a new task.
    reset_controls -- Reset all the control commands to zero.
    get_cuda_functions -- Generate and compiles CUDA code.
    debug_printout -- Give a nice printout of the generated CUDA code with line numbers.
    params_to_cuda -- Transfer cost, policy, and dynamics parameters to device (GPU) memory.
    numerical_check -- Check for Nans/Infs after transferring variables from device to host memory.
    rollouts -- Sample and evaluate trajectories and compute the weighted average over control sequences.
    spline_controls -- Smooth the resulting control sequence by fitting a spline.
    polyfit -- Smooth the resulting control sequence by fitting a polynomial.
    savitsky_galoy --  Smooth the resulting control sequence by Savitsky Galoy filter.
    compute_control -- Given the current state, return an approximation to the optimal controls.
    on_gpu -- Transform numpy array into a gpuarray.
    default_cuda_policy -- CUDA code version of default_policy.
    cuda_rollouts -- CUDA code for sampling and evaluating system trajectories.
    """

    def __init__(self,
                 state_dim,
                 control_dim,
                 num_samples,
                 time_horizon,
                 control_freq,
                 exploration_variance,
                 kinematics,
                 state_costs,
                 SG_window,
                 SG_PolyOrder,
                 LogN_info,
                 initialization_policy=None,
                 num_optimization_iterations=1,
                 policy_args=None,
                 block_dim=(32, 1, 1),
                 spline_smoothing=True,
                 lambda_=1.0,
                 cost_range=(-100000000, 10000000000)):
        """
        Initialize class fields and compile and save CUDA functions for later use.

        Arguments:
        state_dim, control_dim -- state and control dimension sizes.
        num_samples -- number of trajectories to sample each timestep.
        time_horizon -- length (in seconds) of each trajectory sample.
        control_freq -- frequency at which control inputs need to be returned.
        natural_variance -- the natural stochastic variance of the control system.
        dynamics -- either cuda code which can be compiled -OR- a tuple/list containing
                    compilable cuda code and a callable which returns a dictionary of parameter
                    name/value pairs. The names should match global constant array
                    names in the returned cuda code.
        state_costs -- same setup as dynamics except the cost instead of the next state is returned.
        SG_window, SG_PolyOrder -- parameters of Savitzky-Golay filter (if used), where
                 SG_window: the length of the filter window,
                 SG_PolyOrder: the order of the polynomial used to fit the samples.
        LogN_info -- list of [dist_type, mu_LogN, std_LogN], where
                     dist_type: 0 (Normal), 1: (Normal & Log-Normal),
                     mu_LogN: "mean" of the log-normal dist,
                     std_LogN: "std of the log-normal dist.

        Keyword arguments:
        initialization_policy -- callable which takes the current state and returns a control input.
        num_optimization_iterations -- number of sampling iterations to run every timestep.
        policy_args -- a tuple/list which consists of (1) a callable which returns a control input
                       given the current state, (2) cuda code which performs the same function, and
                       (3) a param updater which returns a dictionary of parameter names and values. These
                       parameter names should match global constant arrays names in the returned cuda code.
        block_dim -- Dimension of the cuda blocks in the x-direction. (Y and Z are 1 for now.)
        spline_smoothing -- Whether or not to smooth the control sequence after optimization.
        lambda_ -- Temperature of the softmax when computing the weighted average. Zero corresponds to an unweighted
                 average, and infinity corresponds to the max function.
        cost_range -- The range of valid values for the cost function.
        """
        self.state_dim = state_dim
        self.dist_type, self.mu_LogN, self.std_LogN = LogN_info
        self.control_dim = control_dim
        self.block_dim = block_dim
        """ Note that the actual number of sampled trajectories depends on the dimension of Cuda blocks in the x-direction """  
        self.num_samples = block_dim[0] * (
            (num_samples - 1) // self.block_dim[0] + 1)
        self.num_timesteps = int(time_horizon * control_freq)
        # Nominal control sequence.
        self.U = np.zeros((self.num_timesteps, self.control_dim),
                          dtype=np.float32)
        self.last_U = np.zeros((self.num_timesteps, self.control_dim),
                               dtype=np.float32)
        # Nominal sequence of states.
        self.nominal_sequence = np.zeros(self.num_timesteps * self.state_dim,
                                         dtype=np.float32)
        self.dt = 1.0 / control_freq
        # GPU random number generator
        self.generator = XORWOWRandomNumberGenerator()
        # For MPPI
        if self.dist_type == 0:
            du_d = self.generator.gen_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32)
            rospy.loginfo("Trajectories are sampled from Normal dist.")
        # log-MPPI
        elif self.dist_type == 1:
            rospy.loginfo(
                "Trajectories are sampled from Normal & Log-normal dist.")
            du_LogN_d = self.generator.gen_log_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32, self.mu_LogN, self.std_LogN)
            du_d = du_LogN_d * self.generator.gen_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32)
        self.du = du_d.get()
        self.exploration_variance = exploration_variance
        self.num_optimization_iterations = num_optimization_iterations
        self.spline_smoothing = spline_smoothing
        self.lambda_ = lambda_
        self.SG_window = SG_window
        self.SG_PolyOrder = SG_PolyOrder
        self.cost_range = cost_range
        self.control_filter = np.convolve(np.array([0, 0, 1, 0, 0]),
                                          np.array([1, 0, 0]))
        if initialization_policy is None:
            self.initialization_policy = self.default_initialization_policy
        else:
            self.initialization_policy = initialization_policy
        self.param_dict = {
            "lambda_": np.array([self.lambda_]),
            "control_filter": self.control_filter,
            "du": self.du
        }

        # Save the dynamics code and dynamics param updater if there is one to save.
        if (type(kinematics) is list or type(kinematics) is tuple):
            kinematics_code = kinematics[0]
            self.kinematics_param_getter = kinematics[1]
            self.kinematics_arrs = self.kinematics_param_getter()
        else:
            kinematics_code = kinematics
            self.kinematics_param_getter = None
            self.kinematics_arrs = {}

        # Save the costs code and cost param updater
        if (type(state_costs) is list or type(state_costs) is tuple):
            costs_code = state_costs[0]
            self.costs_param_getter = state_costs[1]
            self.costs_arrs = self.costs_param_getter()
        else:
            costs_code = state_costs
            self.costs_param_getter = None
            self.costs_arrs = {}
        
        # Set the importance sampling policy.
        if (policy_args is None):
            self.policy = self.default_policy
            cuda_policy = self.default_cuda_policy()
            self.policy_param_getter = None
            self.policy_arrs = {}
        if (policy_args is not None):
            self.policy = policy_args[0]
            cuda_policy = policy_args[1]
            self.policy_param_getter = policy_args[2]
            self.policy_arrs = self.policy_param_getter()
        
        # Generate and compile CUDA code for sampling and evaluating trajectories.
        self.cuda_functions = self.get_cuda_functions(cuda_policy,
                                                      kinematics_code,
                                                      costs_code)
    """ @brief: The default control policy (All zeros) which the controller samples around """    
    def default_policy(self, x):
        """Returns a zero control command"""
        return np.zeros(self.control_dim)

    def default_initialization_policy(self, x, args=[]):
        """Returns a zero control command"""
        return np.zeros(self.control_dim)

    """ @brief: Allow the controller to iterate to convergence before starting a new task """
    def initialize_controls(self,
                            R,
                            std_n,
                            weights,
                            targets,
                            init_state=None,
                            num_iters=350):
        """
        Allow the controller to iterate to convergence before starting a new task, (i.e. let it sit and
        think for a moment before acting). This is plausible for some tasks (swinging up a cart pole for
        instance), but less so for others. Usage is optional.

        Arguments:
        R -- Control cost matrix.
        std_n -- Standard deviation of the injected control noise.
        weights, targets -- Cost parameters.
        """
        self.U = np.zeros((self.num_timesteps,self.control_dim), dtype = np.float32)
        # self.U = np.random.randn(self.num_timesteps, self.control_dim) * 0.05
        """for i in range(self.num_timesteps):
            self.U[i,:] = self.initialization_policy(init_state)"""
        if init_state is not None:
            state = np.copy(init_state)
            print(state)
            for i in range(num_iters):
                self.rollouts(state, std_n, R, weights, targets)

    """ @brief: Reset the whole control sequence to the initial policy."""           
    def reset_controls(self):
        for i in range(self.num_timesteps):
            self.U[i, :] = self.initialization_policy(np.zeros(self.state_dim))

    """ @brief: Compile the CUDA code and returns a callable function to perform parallel sampling, and 
            returns addresses for CUDA constant arrays U_d and policy_params """
    def get_cuda_functions(self, cuda_policy, kinematics, state_costs):
        """
        Generate compiled cuda code which can be called during optimization.

        Arguments:
        cuda_policy -- string of cuda code for making policy predictions.
        dynamics -- string of cuda code for making dynamics predictions.
        state_costs -- string of cuda code for computing state_costs
        """
        rollout_kernel = self.cuda_headers(
        ) + cuda_policy + kinematics + state_costs + self.cuda_rollouts()
        # First see if the compilation is successful
        try:
            SourceModule(rollout_kernel)
        except pycuda.driver.CompileError:
            """ If compilation is not succesful print the code with linenumbers and the pycuda error message """
            self.debug_printout(rollout_kernel)
        """ If we were successful compile the code, otherwise this will tell us the error
            and linenumber where it occured. """
        mod = SourceModule(rollout_kernel)
        func = mod.get_function("rollout_kernel")
        U_d = mod.get_global("U_d")[0]
        # Get the addresses for policy, dynamics and cost parameters
        policy_adrs = {}
        for arr_name in self.policy_arrs:
            policy_adrs[arr_name] = mod.get_global(arr_name)[0]
        kinematics_adrs = {}
        for arr_name in self.kinematics_arrs:
            kinematics_adrs[arr_name] = mod.get_global(arr_name)[0]
        costs_adrs = {}
        for arr_name in self.costs_arrs:
            costs_adrs[arr_name] = mod.get_global(arr_name)[0]
        return func, U_d, policy_adrs, kinematics_adrs, costs_adrs

    """ @brief: Print out cuda code in an easy to read format"""
    def debug_printout(self, rollout_kernel):
        rollout_kernel_debug = rollout_kernel.split('\n')
        count = 0
        print("CUDA compilation failed")
        print()
        print("=====================================")
        print()
        for line in rollout_kernel_debug:
            sys.stdout.write("%d %s \n" % (count, line))
            count += 1
        print()
        print("=====================================")

    """ @brief: Transfer policy, dynamics, and cost parameters to device memory """    
    def params_to_cuda(self, policy_params_adrs, kinematics_params_adrs,
                       costs_params_adrs):
        # policy_param_getter returns a dict of keynames and arrays
        if (self.policy_param_getter is not None):
            policy_params = self.policy_param_getter()
            for key in policy_params_adrs:
                gpu_arr = self.on_gpu(policy_params[key])
                cuda.memcpy_dtod(policy_params_adrs[key], gpu_arr.ptr,
                                 gpu_arr.nbytes)
        
        # Transfer the kinematics parameters to CUDA constant memory
        if (self.kinematics_param_getter is not None):
            kinematics_params = self.kinematics_param_getter()
            for key in kinematics_params_adrs:
                gpu_arr = self.on_gpu(kinematics_params[key])
                cuda.memcpy_dtod(kinematics_params_adrs[key], gpu_arr.ptr,
                                 gpu_arr.nbytes)

        # Transfer the cost parameters to CUDA constant memory
        if (self.costs_param_getter is not None):
            costs_params = self.costs_param_getter()
            for key in costs_params_adrs:
                gpu_arr = self.on_gpu(costs_params[key])
                cuda.memcpy_dtod(costs_params_adrs[key], gpu_arr.ptr,
                                 gpu_arr.nbytes)

    """ @brief: Check if any returned values have Nans/Infs """ 
    def numerical_check(self, costs, control_variations):
        # Do some NaN/Infinity checking
        fail = False
        if np.isnan(np.sum(costs)):
            warnings.warn("Nan Deteced in Costs", UserWarning)
            indices = np.isfinite(costs)
            for i in range(self.num_samples):
                if (not indices[i]):
                    costs[i] = float("inf")
            fail = True
        if (np.isnan(np.sum(control_variations))):
            control_variations = np.nan_to_num(control_variations)
            warnings.warn("Nan/Infinity deteced in control variations",
                          UserWarning)
            fail = True
        if (np.sum(costs) == 0):
            warnings.warn("Normalizer is zero", UserWarning)
            fail = True
        return fail

    """ @brief: Rollouts computes an update to the internal control sequence maintained by the controller """
    def rollouts(self, state, std_n, R, weights, targets, cost_baseline=None):
        """
        Generate and evaluate trajectories on the GPU. Then compute a reward weighted average
        of the trajectories and update the nominal control sequence according to the path
        integral update law.

        Arguments:
        state -- The current state.
        R -- Control cost matrix.
        std_n -- Standard deviation of the injected control noise.
        weights, targets -- Cost parameters.

        Returns:
        normalizer -- The sum of the exponentiated costs of the trajectories. This is very high when the
        controller is performing well, and small otherwise. Normalizer < 1 is a bad sign.
        min_cost -- The minimum cost over all the trajectories.
        """
        self.lambda_ = self.param_dict["lambda_"][0]
        if self.dist_type == 0:
            du_d = self.generator.gen_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32)
        elif self.dist_type == 1:
            du_LogN_d = self.generator.gen_log_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32, self.mu_LogN, self.std_LogN)
            du_d = du_LogN_d * self.generator.gen_normal(
                self.num_samples * self.num_timesteps * self.control_dim,
                np.float32)

        self.du = du_d.get()

        self.control_filter = self.param_dict["control_filter"]
        # Transfer arrays to GPU
        std_nd = self.on_gpu(std_n, dtype=np.float32)
        R_d = self.on_gpu(R, dtype=np.float32)
        du_d = self.on_gpu(self.du, dtype=np.float32)
        state_d = self.on_gpu(state)
        costs_d = gpuarray.zeros(self.num_samples, dtype=np.float32)
        nominal_sequence_d = gpuarray.zeros(self.num_timesteps *
                                            self.state_dim,
                                            dtype=np.float32)
        U_d = self.on_gpu(self.U)
        weights_d = self.on_gpu(weights)
        targets_d = self.on_gpu(targets)
        # Unpack CUDA functions and addresses
        rollout_kernel, U_d_adr, policy_params_adrs, kinematics_params_adrs, costs_params_adrs = self.cuda_functions
        # Transfer current control sequence to CUDA constant memory
        cuda.memcpy_dtod(U_d_adr, U_d.ptr, U_d.nbytes)
        # Transfer the policy parameters to CUDA constant memory
        self.params_to_cuda(policy_params_adrs, kinematics_params_adrs,
                            costs_params_adrs)
        # Set blocksize and gridsize for rollout and cost-to-go kernels
        blocksize = self.block_dim
        gridsize = ((self.num_samples - 1) // self.block_dim[0] + 1, 1, 1)
        # Launch the kernel for simulating rollouts
        rollout_kernel(costs_d,
                       du_d,
                       state_d,
                       std_nd,
                       R_d,
                       weights_d,
                       targets_d,
                       nominal_sequence_d,
                       grid=gridsize,
                       block=blocksize)
        cuda.Context.synchronize()
        # Retrieve the costs and control variations from the GPU
        costs = costs_d.get()
        control_variations = du_d.get().reshape(
            (self.num_samples, self.num_timesteps * self.control_dim)).T
        self.nominal_sequence = nominal_sequence_d.get().reshape(
            (self.num_timesteps, self.state_dim))
        # Compute the control update
        min_cost = np.min(costs)
        if (cost_baseline is None):
            cost_baseline = min_cost

        transformed_costs = np.exp(-(1.0 / self.lambda_) *
                                   (costs - cost_baseline))
        normalizer = np.sum(transformed_costs)

        costs = transformed_costs

        fail = self.numerical_check(costs, control_variations)
        costs /= normalizer
        control_variations *= costs
        control_update = np.sum(control_variations, axis=1).reshape(
            (self.num_timesteps, self.control_dim))
        control_update = control_update.reshape(
            (self.num_timesteps, self.control_dim))
        # control_update = self.spline_controls(control_update)
        control_update = self.savitsky_galoy(control_update)
        # control_update = self.polyfit(control_update)
        if np.isfinite(control_update).all():
            self.U = control_update.reshape(
                (self.num_timesteps, self.control_dim))
        return normalizer, min_cost

    """@brief: Smooth the resulting control sequence by Savitsky Galoy filter"""    
    def savitsky_galoy(self, control_update):
        new_update = np.zeros_like(control_update)
        for i in range(self.control_dim):
            new_update[:, i] = scipy.signal.savgol_filter(control_update[:, i],
                                                          self.SG_window,
                                                          self.SG_PolyOrder,
                                                          mode='mirror')
        return new_update

    """@brief: Smooth the resulting control sequence by fitting a polynomial """
    def polyfit(self, control_update):
        new_update = np.zeros_like(control_update)
        for i in range(self.control_dim):
            # fit = np.polyfit(range(len(new_update)), control_update[:,i], 2) # get the coeff. of the poly (2nd order)
            fit = np.polyfit(range(len(new_update)), control_update[:, i],
                             5)  # get the coeff. of the poly (3nd order)
            for j in range(len(new_update)):
                # new_update[j,i] = fit[0]* np.square(j) + fit[1]*j + fit[2] # 2nd order poly
                # new_update[j,i] = fit[0] * np.square(j) * j + fit[1] * np.square(j) + fit[2] * j + fit[3] # 3rd order poly
                new_update[j, i] = fit[0] * np.square(j) * np.square(
                    j) * j + fit[1] * np.square(j) * np.square(
                        j) + fit[2] * np.square(j) * j + fit[3] * np.square(
                            j) + fit[4] * j + fit[5]  # 5rd order poly
        return new_update

    """@brief: Smooth the resulting control sequence by fitting a spline """ 
    def spline_controls(self, control_update, spline_pwr=3):
        """Fits a spline to the current nominal control sequence"""
        knots = np.linspace(0, self.num_timesteps,
                            int((self.num_timesteps * self.dt) / .1))[1:-1]
        old_update = np.copy(control_update)
        for i in range(self.control_dim):
            spline_params = interpolate.splrep(range(self.num_timesteps),
                                               old_update[:, i],
                                               k=spline_pwr,
                                               t=knots)
            control_update[:, i] = interpolate.splev(range(self.num_timesteps),
                                                     spline_params)
        return control_update

    """@brief: Compute an approximation to the optimal control """   
    def compute_control(self, state, cost_params, cost_baseline=None):
        """

        Arguments:
        state -- The current state 
        cost_params -- tuple/list containing R (control cost matrix), std_n (exploration standard deviation),
                       and the weights and targets (cost parameters).

        Keywards Arguments:
        cost_baseline -- Value to subtract from the costs when computing the control update. If None
                         then the minimum value of all the sampled trajectories is used.

        Returns:
        u - control to execute.
        normalizer - sum of the exponentiated trajectory costs.
        min_cost - the minimum cost over all the trajectories.
        """
        std_n, R, weights, targets = cost_params
        for i in range(self.num_optimization_iterations):
            normalizer, min_cost = self.rollouts(state,
                                                 std_n,
                                                 R,
                                                 weights,
                                                 targets,
                                                 cost_baseline=cost_baseline)
        u = self.U[0, :]
        self.last_U = np.copy(self.U)
        # Slide the control sequence down one-timestep
        U_new = np.zeros_like(self.U)
        for i in range(self.control_dim):
            U_new[:, i] = np.convolve(self.U[:, i], self.control_filter)[3:-3]
        U_new[-1, :] = self.initialization_policy(state)
        self.U = U_new
        # Update and compute the control to be excuted
        u += self.policy(state.reshape((1, self.state_dim))).flatten()
        return u, normalizer, min_cost

    """@brief: Transform a numpy array into a gpuarray """    
    def on_gpu(self, a, dtype=np.float32):
        a = a.flatten()
        a = np.require(a, dtype=dtype, requirements=['A', 'O', 'W', 'C'])
        a_d = gpuarray.to_gpu(a)
        return a_d

    """@brief: Cuda code for the default return zero policy."""    
    def default_cuda_policy(self):
        default_policy_template = Template("""

        __device__ void compute_policy(float* input, float* u, int input_dim, int output_dim)
        {
            int i;
            for (i = 0; i < output_dim; i++) {
                u[i] = 0;
            }
        }

        """)
        return default_policy_template.render()

    """@brief: Headers to include for cuda compilation """
    def cuda_headers(self):
        header_template = Template("""                       
        #include <math.h>
        #include <stdio.h>
        #include <float.h>  // import FLT_EPSILON
        
        __device__ __constant__ float U_d[{{timesteps}}*{{control_dim}}];
        """)
        return header_template.render(timesteps=self.num_timesteps,
                                      control_dim=self.control_dim)

    """@brief: Path integral control cuda code for sampling and evaluating trajectories """
    def cuda_rollouts(self):
        rollout_kernel_template = Template("""
        __device__ float get_control_costs(float* u, float* du, float* R)
        {
            int i;
            float cost = 0;
            for (i = 0; i < {{control_dim}}; i++) {
                cost += 0.5*(1 - (1.0/{{exploration_variance}}))*(R[i]*du[i]*du[i]) + R[i]*du[i]*u[i] + 0.5*R[i]*u[i]*u[i];
            }
            return cost;
        }

        __global__ void rollout_kernel(float* costs_d, float* ran_vec, float* init_state,
                                       float* std_nd, float* R_d, float* weights, float* targets, float* nominal_seq_d)
        {
            //Get thread and block index
            int tdx = threadIdx.x;
            int bdx = blockIdx.x;
            int tdy = threadIdx.y;

            //Initialize block wide state and control variables
            __shared__ float state_shared[{{BLOCK_DIM_X}}*({{state_dim}} + {{control_dim}})];
            __shared__ float control_var_shared[{{BLOCK_DIM_X}}*{{control_dim}}];
            __shared__ float std_n[{{control_dim}}];
            __shared__ float R[{{control_dim}}];

            //Initialize local state
            float *s, *u, *du;

            //Initialize local state and control variables
            s = &state_shared[tdx*({{state_dim}} + {{control_dim}})];
            u = &state_shared[tdx*({{state_dim}} + {{control_dim}}) + {{state_dim}}];
            du = &control_var_shared[tdx*{{control_dim}}];

            //Initialize trajectory cost
            float running_cost = 0;
            float cost = 0;

            // Load std_n, R, and the initial state
            for (int i = tdy; i < {{control_dim}}; i+= blockDim.y)
            {
                std_n[i] = std_nd[i];
                R[i] = R_d[i];
            }

            for (int i = tdy; i < {{state_dim}}; i+=blockDim.y)
            {
                s[i] = init_state[i];
            }

            __syncthreads();

             /*<----Start of simulation loop (i.e., the main program loop) -----> */
            for (int i = 0; i < {{timesteps}}; i++)
            {
                // Get the initial control estimate from the feedback controller
                compute_policy(s, u, {{state_dim}}, {{control_dim}});
                __syncthreads();

                // Get the control and control variation
                for (int j = tdy; j < {{control_dim}}; j+=blockDim.y)
                {
                    u[j] += U_d[i*{{control_dim}} + j];
                    // Noise free rollout
                    if ((tdx == 0 && bdx == 0))
                    {
                        du[j] = 0;
                    }
                    else
                    {
                        du[j] = std_n[j]*ran_vec[(blockDim.x*bdx + tdx)*{{timesteps}}*{{control_dim}} + i*{{control_dim}} + j];
                    }
                }

                // Save the random control variation (i.e.e, control updates)
                for (int j = tdy; j < {{control_dim}}; j+= blockDim.y)
                {
                    ran_vec[(blockDim.x*bdx + tdx)*{{timesteps}}*{{control_dim}} + i*{{control_dim}} + j] = u[j] + du[j];
                }

                // Save the nominal sequence
                if (tdx == 0 && bdx == 0 && tdy == 0)
                {
                    for (int j = 0; j < {{state_dim}}; j++)
                    {
                        nominal_seq_d[i*{{state_dim}} + j] = s[j];
                    }
                }
                __syncthreads();


                if (tdy == 0){
                    kinematics(s, u, du, {{dt}}, i);
                }
                __syncthreads();

                // Get state and control costs
                if (tdy == 0)
                {
					running_cost = get_state_cost(s, weights, targets, i);
                    if (isnan(running_cost) || running_cost > {{max_cost}})
                    {
                        running_cost = {{max_cost}};
                    }
                    else if (running_cost < {{min_cost}})
                    {
                        running_cost = {{min_cost}};
                    }
                    cost += running_cost;
                    cost += get_control_costs(u, du, R);
                }
                __syncthreads();
            }
            /* <------- End of the simulation loop ----------> */

            // Write back the cost results to global memory.
            if (tdy == 0)
            {
                costs_d[(blockDim.x*bdx + tdx)] = cost*{{dt}};
            }
        }
        """)
        return rollout_kernel_template.render(
            dt=self.dt,
            state_dim=self.state_dim,
            control_dim=self.control_dim,
            timesteps=self.num_timesteps,
            num_rollouts=self.num_samples,
            exploration_variance=self.exploration_variance,
            max_cost=self.cost_range[1],
            min_cost=self.cost_range[0],
            lambda_=self.lambda_,
            BLOCK_DIM_X=self.block_dim[0])
