# log-MPPI_ros

This repository contains the ROS implementation of the Model Predictive Path Integral (MPPI) and log-MPPI control strategy using the [PyCUDA](https://documen.tician.de/pycuda/) module (namely, Python wrapper for Nvidia CUDA), with the aim of performing autonomous navigation of Autonomous Ground Vehicles (AGVs) in unknown cluttered environments. In this work, the 2D grid map is incorporated, as a local costmap, into the MPPI algorithm for performing collision-free navigation in either static or dynamic unknown cluttered environments.
  
`log-MPPI_ros` is an extension of the MPPI control algorithm proposed by Williams et al. in [this article](https://arc.aiaa.org/doi/pdf/10.2514/1.G001921). The key idea of log-MPPI is that the trajectories are sampled from the product of normal and log-normal distribution (namely, NLN mixture), instead of sampling from only Gaussian distribution. 

With such a sampling strategy, we provide more efficient trajectories than the vanilla MPPI variants, ensuring a much better exploration of the state-space of the given system and reducing the risk of getting stuck in local minima. Moreover, a small injected noise variance can be utilized so that violating system constraints can be avoided.

![](media/demo_mppi.gif "MPPI: 0.1 trees/m^2 forest-like env") 
![](media/demo_log-mppi.gif "log-MPPI: 0.2 trees/m^2 forest-like env")

## Update (Incorporating log-MPPI into mppic, ROS2):
We are pleased to note that our proposed **log-MPPI** is now included in the [mppic](https://github.com/artofnothingness/mppic) repository. To the best of our knowledge, this repo marks the first `CPU-based MPPI implementation` that can be compiled with **ROS2** and has been optimized using vectorization and tensor operations. We extend our sincere thanks to [Steve Macenski](https://github.com/SteveMacenski) for his contribution. Link to `logmppi` Branch: https://github.com/artofnothingness/mppic/tree/logmppi
 
## Paper:

If you find this code useful in your research, please cite:

I. S. Mohamed, K. Yin and L. Liu, "Autonomous Navigation of AGVs in Unknown Cluttered Environments: Log-MPPI Control Strategy," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 10240-10247, Oct. 2022.

Paper: https://ieeexplore.ieee.org/document/9834098 \
Video: https://youtu.be/_uGWQEFJSN0

Bibtex:
```
@article{mohamed2022autonomous,
  author={Mohamed, Ihab S. and Yin, Kai and Liu, Lantao},
  journal={IEEE Robotics and Automation Letters}, 
  title={Autonomous Navigation of AGVs in Unknown Cluttered Environments: Log-MPPI Control Strategy}, 
  year={2022},
  volume={7},
  number={4},
  pages={10240-10247},
  doi={10.1109/LRA.2022.3192772}
}
```
## Recent Work:
Please find our more recent work in the following links:
1. GP-MPPI: https://github.com/IhabMohamed/GP-MPPI
2. U-MPPI: https://github.com/IhabMohamed/U-MPPI

## Software Requirements:
* ROS installation
* Ubuntu
* CUDA installation (In my opinion, the `deb (local)` option could be easier [[LINK](https://developer.nvidia.com/cuda-downloads?target_os=Linux)]).
	* Once CUDA is installed, the following lines should be added to `~/.bashrc`
	
	```bash
	# Note that {cuda} should be replaced with the installed version on your machine, e.g., cuda-11
	export CUDA_HOME=/usr/local/{cuda}
	export LD_LIBRARY_PATH=${CUDA_HOME}/lib64:$LD_LIBRARY_PATH
	export PATH=${CUDA_HOME}/bin:${PATH} 
	```
	* Then, source bash configuration `source ~/.bashrc`
* PyCUDA installation [[LINK](https://pypi.org/project/pycuda/)]
* Jackal-specific metapackages installation for desktop and simulation [[LINK](https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html)]. Since we have done some modifications to those packages, it is highly recommended to use the provided packages in [jackal_ros](jackal_ros) directory.
* Pedestrian Simulator (Only if you wish to test the performance with dynamic environment): pedsim_ros: https://github.com/bbrito/pedsim_ros 

## Installation Instructions:
Our proposed control strategies were tested with (i) ROS-Kinetic on Ubuntu 16.04, (ii) ROS-Noetic on Ubuntu 20.04, and (iii) GPU: NVIDIA GeForce GTX 1050 Ti and NVIDIA GeForce GTX 1660 Ti. 

We assume that you already have a complete ROS installation, with a ROS workspace entitled `catkin_ws`.
Please follow the following instructions: 
1. Clone the repo and its dependency into the workspace 
	```
	cd catkin_ws/src
	git clone git@github.com:IhabMohamed/log-MPPI_ros.git
	```
2. Then, build it
	```
	cd ../
	catkin_make
	source devel/setup.bash
	```

## ROS Packages:
### jackal_ros
The `jackal_ros` package contains the common packages for Jackal, including Gazebo simulator with various world definitions and their ROS launch files, robot description, messages, and controllers. 

### mission_control
This package is forked from [ethz-asl/deep_motion_planning](https://github.com/ethz-asl/deep_motion_planning) with very few modifications. It contains a mission control node that executes a user-defined mission. Therefore, a txt file is parsed and a sequence of commands is generated. This sequence is then processed step-by-step. For more details on the definition of a mission, please refer to the [README](mission_control/README.md) file in the package directory.

### mppi_control
This node contains the implementation of both vanilla MPPI and log-MPPI control strategies. It subscribes to: (i) the 2D local costmap built by the robot on-board sensor for achieving a collision-free navigation and (ii) the set of desired poses published by the `mission_control` node; then, consequently, it publishes the control commands, namely, the linear and angular velocities of the robot.

### velodyne_simulator
URDF description and Gazebo plugins to simulate Velodyne laser scanners. If you encountered a very slow-motion of the robot in Gazebo, there are two ways to tackle this issue:
(i) you should be sure that the `gpu` parameter in [VLP-16.urdf.xacro](velodyne_simulator/velodyne_description/urdf/VLP-16.urdf.xacro) is set to `true` (we actually prefer this option), OR (ii) you can decrease the number of `samples` to let's say 500 instead of 1875 in [VLP-16.urdf.xacro](velodyne_simulator/velodyne_description/urdf/VLP-16.urdf.xacro). 

## Usage: 

1. In order to start the Gazebo simulation and Jackal within the given forest-like environment, run:
	```
	roslaunch jackal_gazebo world_stage.launch env_name:="forest0"
	```
	* `forest0` is a 50 meters by 50 meters, with a density of 0.1 trees/square meter; `forest1` has the same size with a density of 0.2 trees/square meter. To generate your own forest-like environment, please refer to [forest_gen](https://github.com/ethz-asl/forest_gen) package.

2. To start the MPPI or log-MPPI controller, 2D costmap node, and visualization of the system, run:
 	* For MPPI, run:
	```
	roslaunch mppi_control control_stage.launch normal_dist:=true
	```
	* For log-MPPI, run:
	```
	roslaunch mppi_control control_stage.launch normal_dist:=false
	```

3. Now, you can start a mission by executing:
	```
	roslaunch mission_control forest0_mission.launch
	```

### Primary code maintainer:
Ihab S. Mohamed (e-mail: mohamedi@iu.edu)\
Vehicle Autonomy and Intelligence Lab ([VAIL](https://vail.sice.indiana.edu/))\
Indiana University - Bloomington, USA

