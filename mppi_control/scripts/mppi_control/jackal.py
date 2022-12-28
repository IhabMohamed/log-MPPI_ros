"""
@author: Ihab S. Mohamed, Vehicle Autonomy and Intelligence Lab - Indiana University, Bloomington
"""
"""
@brief: The  definition of the kinematics model of a differential wheeled robot, e.g., Jackal Robot,
as well as the definition of the state-dependent running cost function. 
"""
import numpy as np
from jinja2 import Template
import matplotlib.pyplot as plt

class Jackal:
    def __init__(self,
                 state_dim,
                 dt,
                 max_linear_velocity,
                 max_angular_velocity,
                 map_info,
                 seed=123):
        self.state_dim = state_dim
        self.dt = dt
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

        self.map_size = map_info["map_size"]
        self.local_costmap_origin_x = np.array([map_info["costmap_origin_x"]])
        self.local_costmap_origin_y = np.array([map_info["costmap_origin_y"]])
        self.local_costmap_origin = np.array(
            [self.local_costmap_origin_x, self.local_costmap_origin_y])
        self.local_costmap_resolution = map_info["costmap_resolution"]
        self.collision_cost = map_info["collision_cost"]
        self.footprint = map_info["footprint"]

        self.obstacle_grid = np.zeros(self.map_size * self.map_size,
                                      dtype=np.float32)
        ''' \param "self.Control_Constraints = true" means that the control constraints are considered
            in the control law design. More precisely, an element-wise clamping function is used to restrict
            the control input to remain within a given range, for all samples drawn from the dynamics system'''
        self.Control_Constraints = 'true'
        '''Since the control constraints, in this case, acting as soft constraints. It is notoriously difficult to ensure
            that the control input, obtained by the controller, remains always within its allowed bounds, even after rejecting
            each trajectory that violates the control input limits. For this reason, \param "self.ctrl_scale" is here used. 
            Another way is to apply the clamping function to the optimal control sequence obtained by the controller'''
        self.ctrl_scale = 0.6

        self.state = np.zeros(self.state_dim, dtype=np.float32)

    ''' @brief: Updatting the robot state (x, y, theta) '''    
    def update_state(self, s, u):
        self.kinematics(s, u)
        self.state = s
        return s

    def kinematics(self, s, u):
        # Set the control input: linear and angular velocities
        v, w = u
        # Update x, y, yaw of the vehicle
        s[0] += np.cos(s[2]) * v * self.dt
        s[1] += np.sin(s[2]) * v * self.dt
        s[2] += w * self.dt

    ''' @brief: Returning the next state of the vehicle (x, y, theta) '''    
    def update_kinematics(self, s, u):
        # Set the control input: linear and angular velocities
        v, w = u
        # Update x, y, yaw of the vehicle
        s[0] += np.cos(s[2]) * v * self.dt
        s[1] += np.sin(s[2]) * v * self.dt
        s[2] += w * self.dt
        return s

    ''' @brief: Calculating the running state and control costs ''' 
    def cost(self, s, u, cost_params):
        weights, targets, R = cost_params
        state_cost = np.sum(weights * (s - targets)**2)
        x_idx = int((s[0] - self.local_costmap_origin_x) /
                    self.local_costmap_resolution)
        y_idx = int((s[1] - self.local_costmap_origin_y) /
                    self.local_costmap_resolution)
        if (x_idx >= 0 and y_idx >= 0 and x_idx < self.map_size
                and y_idx < self.map_size
                and self.obstacle_grid[y_idx, x_idx] > 0):
            state_cost = float("inf")

        control_cost = np.sum(0.5 * u * R * u)
        return state_cost, control_cost

    def plot_grid(self, results_rootpath, counter):
        plt.imshow(self.obstacle_grid, interpolation="nearest")
        plt.savefig(results_rootpath + '/costmap_{n}.png'.format(n=counter))
        np.savetxt(results_rootpath + '/costmap.csv',
                   self.obstacle_grid,
                   fmt='%s')
        # plt.show()

    def param_getter(self):
        return {
            "obstacle_grid": self.obstacle_grid,
            "origin": self.local_costmap_origin
        }

    ''' @brief: Updatting the local costmap each timestep '''    
    def update_obstacle_grid(self, local_costmap, costmap_updated_origin):
        self.obstacle_grid = local_costmap
        self.local_costmap_origin_x = np.array([costmap_updated_origin[0]])
        self.local_costmap_origin_y = np.array([costmap_updated_origin[1]])
        self.local_costmap_origin = costmap_updated_origin

    ''' @brief: The CUDA code of the robot kinematics '''     
    def cuda_kinematics(self):
        kinematics_template = Template("""
        __device__ void kinematics(float* s, float* u, float* du, float dt, int timestep)
        {

            // The command is given as (u[0] + du[0]) for u0, and so on.
            float v = (u[0] + du[0]);
            float w = (u[1] + du[1]);

            float ctrl_scale = {{ctrl_scale}};
            bool Ctrl_Const = {{Control_Constraints}};

           // Handling Control Constraints: the maximum linear and angular velocities (clamping function)
            if (Ctrl_Const)
            {
                if (v > {{max_linear_velocity}}){
                    v = ctrl_scale*{{max_linear_velocity}};
                }
                if (v < - {{max_linear_velocity}}){
                    v = - ctrl_scale*{{max_linear_velocity}};
                }
                //------------------------------------------------------------
                if (w > {{max_angular_velocity}}){
                    w = ctrl_scale*{{max_angular_velocity}};
                }
                if (w < - {{max_angular_velocity}}){
                    w = - ctrl_scale*{{max_angular_velocity}};
                }
            }

            s[0] += __cosf(s[2])* v *dt;
            s[1] += __sinf(s[2])* v *dt;
            s[2] += w * dt;
         }
         """)
        return kinematics_template.render(
            max_linear_velocity=self.max_linear_velocity,
            max_angular_velocity=self.max_angular_velocity,
            Control_Constraints=self.Control_Constraints,
            ctrl_scale=self.ctrl_scale)

    ''' @brief: The CUDA code of the state-dependent cost and collision indicator function '''     
    def cuda_state_cost(self):
        state_cost_template = Template("""
            __device__ float obstacle_grid[{{map_size}}*{{map_size}}];
            __device__ __constant__ float origin[2];

            /* 
            @brief: multiply(): for computing the multiplication of two matrices
                where: 
                    m, n: the number of rows and columns for matrix A
                    p: the number of columns for matrix B
                    Note that: the number of columns of A  must be qual to the number of rows of B 
            */
            __device__ void multiply(float* mul, float* A, float* B, int m, int n, int p)
            {
                
                for(int i=0; i<m; i++)
                {
                    for(int j=0; j<p; j++)
                    {
                        mul[p*i+j] = 0;
                        for(int k=0;k<n; k++)
                        {
                            mul[p*i+j] = mul[p*i+j] + A[n*i+k] * B[p*k+j];
                        }
                    }
                }
            }

            /*
            @brief: sumMat(): for calculating the sum of two matrix, 
                where r: the number of rows,
                      c: the number of columns.
            */
            __device__ void sumMat(float *sum_, float *A, float *B, int r, int c)
            {
                for (int i = 0; i < c*r; ++i)
                    sum_[i] = A[i] + B[i];
            }

            /*
            @brief: get_crash(): a collision indicator function for checking out the collision with obstacles 
                based on the local costmap (i.e., 2D grid map) built by the robot on-board sensor.
            */
            __device__ int get_crash(float* s){
                int crash = 0;
                
                float origin_x = origin[0];
                float origin_y = origin[1];

                float c_theta = __cosf(s[2]);
                float s_theta = __sinf(s[2]);

                float mult_a[3], mult_b[3], mult_c[3], mult_d[3];
                int mx, my;
                float resolution = {{resolution}};
                
                /*  Robot's footprint in the Robot frame, assuming the surface area occupied by the robot
                    is rectangular-shaped area, where a[] & b[] refer to the robot's width in x-axis in meters, and
                    c[] & d[] refer to the robot's height in y-axis in meters.  
                */  
                float a[] = { {{footprint}} , 0, 0};
                float b[] = { -{{footprint}}, 0, 0};
                float c[] = {0, {{footprint}}, 0};
                float d[] = {0, -{{footprint}}, 0};
                
                // Rotation matrix between world and robot frames
                float rot[] = {c_theta, s_theta, 0,
                             -s_theta, c_theta, 0,
                              0,      0,   1};

                // Transfer the robot's footprint into world frame
                multiply(mult_a, rot, a, 3, 3, 1);
                multiply(mult_b, rot, b, 3, 3, 1);
                multiply(mult_c, rot, c, 3, 3, 1);
                multiply(mult_d, rot, d, 3, 3, 1);

                sumMat(a, mult_a, s, 3, 1);
                sumMat(b, mult_b, s, 3, 1);
                sumMat(c, mult_c, s, 3, 1);
                sumMat(d, mult_d, s, 3, 1);

                // Check the collision with obstacles                
                mx = int((a[0] - origin_x) / resolution);
                my = int((a[1] - origin_y) / resolution);
                if (obstacle_grid[{{map_size}}*my + mx] > 0){
                    crash = 1;
                }

                mx = int((b[0] - origin_x) / resolution);
                my = int((b[1] - origin_y) / resolution);
                if (obstacle_grid[{{map_size}}*my + mx] > 0){
                    crash = 1;
                }

                mx = int((c[0] - origin_x) / resolution);
                my = int((c[1] - origin_y) / resolution);
                if (obstacle_grid[{{map_size}}*my + mx] > 0){
                    crash = 1;
                }

                mx = int((d[0] - origin_x) / resolution);
                my = int((d[1] - origin_y) / resolution);
                if (obstacle_grid[{{map_size}}*my + mx] > 0){
                    crash = 1;
                }
                return crash;
            }

            /*
            @brief: get_state_cost(): for computing the state-dependent running cost. 
            */
            __device__ float get_state_cost(float* s, float* weights, float* targets, int timestep)
            {
                float state_cost = 0;

                for (int i = 0; i < {{state_dim}}; i++) {
                    state_cost += weights[i]*(s[i] - targets[i])*(s[i] - targets[i]);
                }

                int x_idx, y_idx;
                float origin_x = origin[0];
                float origin_y = origin[1];
                
                x_idx = int((s[0] - origin_x ) / {{resolution}} );
                y_idx = int((s[1] - origin_y ) / {{resolution}} );
                
                if (x_idx >= 0 && y_idx >= 0 && x_idx < {{map_size}} && y_idx < {{map_size}} && (get_crash(s) > 0)){
                    state_cost += {{collision_cost}};
                }
               return state_cost;
            }
            """)
        return (state_cost_template.render(
            state_dim=self.state_dim,
            footprint=self.footprint,
            map_size=self.map_size,
            collision_cost=self.collision_cost,
            resolution=self.local_costmap_resolution), self.param_getter)
