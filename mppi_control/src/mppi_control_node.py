#!/usr/bin/env python3
"""
@author: Ihab S. Mohamed, Vehicle Autonomy and Intelligence Lab - Indiana University, Bloomington, USA
"""
"""
@brief: The main ROS node for performing autonomous navigation of a differential wheeled robot (e.g., ClearPath Jackal robot)
based on MPPI and log-MPPI, assuming that the map (i.e., 2d_costmap) is genereted online using the onboard sensor.
"""
import datetime
import os
import sys
import time
import numpy as np
import rospy

import actionlib
# Import MPPI, Jackal Math. models, and utils
import mppi_control.utils as uls
from mppi_control.jackal import Jackal
from mppi_control.mppi import MPPI_Controller

# ROS messages
from geometry_msgs.msg import Point, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class mppiControllerNode():

    def __init__(self):
        rospy.init_node('mppi_control_node', anonymous=True)

        # Get the control frequency
        self.hz = np.array(rospy.get_param("~samplingRate"))
        self.rate = rospy.Rate(self.hz)

        # To print out the current state and the optimal control generated by the controller
        self.print_out = rospy.get_param("~print_out")

        # Retrieve the parameters of the costmap
        self.costmap_size = rospy.get_param("~costmap_size")
        self.local_costmap_resolution = rospy.get_param("~costmap_resolution")
        self.local_costmap_Origin_x = rospy.get_param("~costmap_origin_x")
        self.local_costmap_Origin_y = rospy.get_param("~costmap_origin_y")
        self.collision_cost = rospy.get_param("~collision_cost")
        self.footprint = rospy.get_param("~footprint")

        # \param "without_heading" to navigate without taking into account the heading of the robot, since we have 360 [deg.] LiDaR
        self.without_heading = rospy.get_param("~without_heading")
        if self.without_heading:
            rospy.loginfo(
                "Headless navigation task is being performed, ENJOY ;-)")
        else:
            rospy.loginfo(
                "The navigation task is being performed taken into account the robot heading, ENJOY ;-)"
            )

        """ ROS publishers and subscribers """
        self.Odometry_sub = rospy.Subscriber("odom",
                                             Odometry,
                                             self.OdometryCallback,
                                             queue_size=10)

        # To subscribe to the published target pose by the move_base package
        self.goal_sub = rospy.Subscriber('goal', PoseStamped,
                                         self.goal_topic_callback)

        # To subscribe to the local costmap published by move_base (namely, costmap_2d)
        self.local_costmap_sub = rospy.Subscriber("local_costmap",
                                                  OccupancyGrid,
                                                  self.localCostmapCallback,
                                                  queue_size=10)

        self.cmd_vel_pub = rospy.Publisher("mppi/cmd_vel", Twist, queue_size=1)
        self.predicted_path_pub = rospy.Publisher("visualization_marker",
                                                  Marker,
                                                  queue_size=1)

        # Connect to the action API of the move_base package
        self._as = actionlib.SimpleActionServer('move_base',
                                                MoveBaseAction,
                                                auto_start=False)
        self._as.register_goal_callback(self.goal_callback)
        self._as.register_preempt_callback(self.preempt_callback)
        # Explicitly start the action server, "auto_start" is set to false.
        self._as.start()

        self.navigation_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        while not self.navigation_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo('Waiting for MPPI_move_base action server')

        # Setup regular publisher of Twist message
        self.cmd_vel_msg = Twist()

        # Get the states and the control input dimensions
        self.state_dim = np.array(rospy.get_param("~state_dim"))
        self.control_dim = np.array(rospy.get_param("~control_dim"))

        # Initialize the states and desired states
        self.state = np.zeros(self.state_dim, dtype=np.float32)
        self.targets = np.zeros(self.state_dim, dtype=np.float32)

        # Get the default desired pose
        self.target_pose = None
        self.desired_pose = np.array(rospy.get_param("~desired_pose"),
                                     dtype=np.float32)
        self.desired_pose[2] = np.deg2rad(self.desired_pose[2])
        self.yaw_desired = self.desired_pose[2]
        self.minimumDistance = np.array(rospy.get_param("~minimumDistance"))
        ''' Reset Gazabo world'''
        '''rospy.wait_for_service('/gazebo/reset_world')
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world() '''

        # Load MPPI controller parameters
        rospy.loginfo("MPPI parameters are loaded..... Enjoy :-)")
        self.time_horizon = np.array(rospy.get_param("~time_horizon"))
        self.exploration_variance = np.array(
            rospy.get_param("~exploration_variance"))
        self.num_trajectories = rospy.get_param("~num_trajectories")
        self.weights = np.array(rospy.get_param("~weights"))
        self.weights = self.weights * np.ones(self.state_dim, dtype=np.float32)
        self.std_n = np.array(rospy.get_param("~std_n"))
        self.lambda_ = rospy.get_param("~lambda")
        self.R = 1 * self.lambda_ / self.std_n
        """ Get the type of the distribution that will be used for updating the control inputs
            self.dist_type = 0, for Normal
            self.dist_type = 1, for Norma and logNormal
        """
        self.dist_type = rospy.get_param("~dist_type")
        # Set the mean and standard deviation of Log-Normal dist based on the corresponding Normal distribution.
        self.mu_LogN, self.std_LogN = uls.Normal2LogN(0, np.mean(self.std_n))
        self.LogN_info = [self.dist_type, self.mu_LogN, self.std_LogN]

        # Get the injected control noise variance
        # For MPPI
        if self.dist_type == 0:
            self.Sigma_du = np.square(self.std_n)
        # For log-MPPI
        else:
            self.Sigma_du = uls.NLN(0, np.square(self.std_n), self.mu_LogN,
                                    np.square(self.std_LogN))[1]

        # Load the parameters of the Savitsky Galoy filter
        self.SG_window = np.array(rospy.get_param("~SG_window"))
        self.SG_PolyOrder = np.array(rospy.get_param("~SG_PolyOrder"))

        # Get the maximum allowable velocities of the robot
        self.max_linear_velocity = np.array(
            rospy.get_param("~max_linear_velocity"))
        self.max_angular_velocity = np.array(
            rospy.get_param("~max_angular_velocity"))

        # Combine the map information in dictionary form
        self.map_info = {
            "map_size": self.costmap_size,
            "costmap_origin_x": self.local_costmap_Origin_x,
            "costmap_origin_y": self.local_costmap_Origin_y,
            "costmap_resolution": self.local_costmap_resolution,
            "collision_cost": self.collision_cost,
            "footprint": self.footprint
        }
        ''' Create a new instance of "Jackal" class and assign this object to the local variable "self.robot" '''
        self.robot = Jackal(self.state_dim, 1 / float(self.hz),
                            self.max_linear_velocity,
                            self.max_angular_velocity, self.map_info)
        ''' Create a new instance of "MPPI_Controller" class and assign this object to the 
            local variable "self.jackal_controller" '''
        self.jackal_controller = MPPI_Controller(self.state_dim,
                                                 self.control_dim,
                                                 self.num_trajectories,
                                                 self.time_horizon,
                                                 self.hz,
                                                 self.exploration_variance,
                                                 self.robot.cuda_kinematics(),
                                                 self.robot.cuda_state_cost(),
                                                 self.SG_window,
                                                 self.SG_PolyOrder,
                                                 self.LogN_info,
                                                 lambda_=self.lambda_)

        # Costs Initialization
        self.state_cost, self.control_cost, self.fail = 0, 0, False
        self.jackal_controller.reset_controls()

        # Repositories for saving the results
        self.state_history, self.desired_state_history, self.control_history = [], [], []
        self.state_cost_history, self.control_cost_history, self.min_cost_history = [], [], []
        self.mppi_time_history = []

        # Create folder for saving data of the running mission
        results_folder = rospy.get_param("~results_folder")
        if results_folder:
            self.results_rootpath = os.path.join(
                results_folder,
                datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
            os.makedirs(self.results_rootpath)
        else:
            self.results_rootpath = None

    def __enter__(self):
        return self

    """ @brief: Callback function for the Odometry message (i.e., robot pose) """
    def OdometryCallback(self, odometry_msg):
        # Get the robot's Pose
        Jackal_quaternion = odometry_msg.pose.pose.orientation
        Jackal_position = odometry_msg.pose.pose.position

        qx, qy, qz, qw = Jackal_quaternion.x, Jackal_quaternion.y, Jackal_quaternion.z, Jackal_quaternion.w
        x, y = Jackal_position.x, Jackal_position.y
        yaw = euler_from_quaternion([qx, qy, qz, qw])[2]
        self.Jackal_states = ([x, y, yaw])

    """ @brief: Callback function when a new goal pose is requested """
    def goal_callback(self):
        self.abort_planning = False
        goal = self._as.accept_new_goal()
        self.target_pose = goal  # target_pose as a ROS message

        x_desired, y_desired = self.target_pose.target_pose.pose.position.x, self.target_pose.target_pose.pose.position.y
        goal_orientation = self.target_pose.target_pose.pose.orientation
        q = [
            goal_orientation.x, goal_orientation.y, goal_orientation.z,
            goal_orientation.w
        ]
        self.yaw_desired = euler_from_quaternion(q)[2]
        # Get the desired_pose as a state array
        self.desired_pose = np.array([x_desired, y_desired, self.yaw_desired],
                                     dtype=np.float32)
        sys.stdout.write(
            "Desired pose: x: %.3f, y: %.3f, yaw: %.3f \n" %
            (x_desired, y_desired, self.yaw_desired * 180 / np.pi))

    """ @brief: Callback function when the current action is preempted """
    def preempt_callback(self):

        rospy.logerr('Action preempted')
        self._as.set_preempted(result=None, text='External preemption')

    """ @brief: Callback function when action message is generated """
    def goal_topic_callback(self, data):
        # Generate a action message
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = data.pose.position.x
        goal.target_pose.pose.position.y = data.pose.position.y

        goal.target_pose.pose.orientation.x = data.pose.orientation.x
        goal.target_pose.pose.orientation.y = data.pose.orientation.y
        goal.target_pose.pose.orientation.z = data.pose.orientation.z
        goal.target_pose.pose.orientation.w = data.pose.orientation.w

        # Send the waypoint
        self.navigation_client.send_goal(goal)

    """ @brief: Callback function when the 2D costmap is updated/published """
    def localCostmapCallback(self, local_costmap_msg):
        # Retrieve the costmap's size in [pixels], data [cost/cell], and its Origin
        self.local_costmap_width = local_costmap_msg.info.width
        self.local_costmap_height = local_costmap_msg.info.height

        # Built a 2D costmap from the given raw data by costmap_2d package
        local_costmap = np.array(local_costmap_msg.data, dtype=np.float32)

        # Get the obstacle grid (i.e., 2D costmap) by directly reshape the given raw data
        local_costmap = local_costmap.reshape(
            (self.local_costmap_width, self.local_costmap_height))
        self.local_costmap = local_costmap

        # Retrieve the updated Origin
        self.local_costmap_updated_origin_x = local_costmap_msg.info.origin.position.x
        self.local_costmap_updated_origin_y = local_costmap_msg.info.origin.position.y
        self.costmap_updated_origin = np.array([
            self.local_costmap_updated_origin_x,
            self.local_costmap_updated_origin_y
        ])
        self.local_costmap_resolution = round(
            local_costmap_msg.info.resolution, 3)

        # Retrieve the Origin when the robot at (0,0)
        self.local_costmap_Origin_x = -self.local_costmap_width * self.local_costmap_resolution / 2
        self.local_costmap_Origin_y = -self.local_costmap_height * self.local_costmap_resolution / 2

    """ @brief: Checking out whether the desired pose is reached or not """
    def check_goal_reached(self, state, target):
        """
        Check if the position and orientation are close enough to the target.
        If this is the case, set the current goal to succeeded.
        """
        position_tolerance = self.minimumDistance - 0.1
        orientation_tolerance = 0.15
        distanceToGoal = np.sqrt(
            np.square(target[1] - state[1]) + np.square(target[0] - state[0]))
        if distanceToGoal < position_tolerance and abs(target[2] - state[2]) < orientation_tolerance:
            self._as.set_succeeded()

    """ @brief: Publisher function for publishing the optimal predicted path obtained by the controller
                for visualization propose """
    def publish_predicted_path(self):
        self.line_strip = Marker()
        self.line_strip.id = 0
        self.line_strip.header.frame_id = "map"
        self.line_strip.header.stamp = rospy.Time.now()
        self.line_strip.type = self.line_strip.LINE_STRIP
        self.line_strip.action = self.line_strip.ADD
        self.line_strip.scale.x = 0.1
        self.line_strip.color.a = 1.0
        self.line_strip.color.b = 1.0
        self.line_strip.pose.orientation.w = 1.0
        self.line_strip.pose.position.x = 0
        self.line_strip.pose.position.y = 0
        self.line_strip.pose.position.z = 0
        self.line_strip.points = []
        for i in range(len(self.jackal_controller.U)):
            self.p = Point()
            u = self.jackal_controller.U[i, :]
            if i == 0:
                state = self.robot.update_kinematics(self.state, u)
            else:
                state = self.robot.update_kinematics(state, u)
            self.p.x = state[0]
            self.p.y = state[1]
            self.p.z = 0
            self.line_strip.points.append(self.p)
        self.predicted_path_pub.publish(self.line_strip)

    """ @brief: Handling the robot's heading angle, based on each quarter, for performing a heading navigation """
    def headingAngle(self, current_state, desired_state):
        delta_x = desired_state[0] - current_state[0]
        delta_y = desired_state[1] - current_state[1]
        desired_heading = np.arctan2(delta_y, delta_x)

        if ((desired_heading - current_state[2]) <
            (-np.pi / 2.0)) and ((desired_heading - current_state[2]) >
                                 (-3.0 * np.pi / 2.0)):
            desired_heading = desired_heading - np.pi
            case = 1
        elif (((desired_heading - current_state[2]) > (np.pi / 2))
              and ((desired_heading - current_state[2]) <
                   (3.0 * np.pi / 2.0))):
            desired_heading = desired_heading + np.pi
            case = 2
        elif ((desired_heading - current_state[2]) < (-3.0 * np.pi / 2.0)):
            desired_heading = desired_heading + 2 * np.pi
            case = 3
        elif ((desired_heading - current_state[2]) > (3.0 * np.pi / 2.0)):
            desired_heading = desired_heading - 2 * np.pi
            case = 4
        else:
            desired_heading = desired_heading
            case = 5
        return desired_heading, case

    """ @brief: The primary function for running the MPPI algorithm """
    def run_mppi(self):
        try:
            # Sleep for 5 seconds to be sure that all sensors are active
            rospy.sleep(2.0)
            if self.Jackal_states != None:
                rospy.loginfo("The MPPI controller got first odometry message")
                self.init_pose = self.Jackal_states

            # Catch the first desired pose, which is the Origin before running the control missions.
            self.targets = self.desired_pose
            while not rospy.is_shutdown():
                """ Update the obstacle map based on the costmap acquired by the sensor """
                self.robot.update_obstacle_grid(self.local_costmap,
                                                self.costmap_updated_origin)
                # Make sure, we have a goal
                if not self._as.is_active():
                    continue

                # Read the current and desired states
                self.state = np.copy(self.Jackal_states)
                self.targets = np.copy(self.desired_pose)

                self.distanceToGoal = np.sqrt(
                    np.square(self.targets[1] - self.state[1]) +
                    np.square(self.targets[0] - self.state[0]))

                if self.without_heading:
                    if self.distanceToGoal > self.minimumDistance:
                        self.targets[2] = self.state[2]
                    else:
                        self.targets[2] = self.yaw_desired
                else:
                    # Calculate the desired yaw angle (keep the robot heading towards the goal)
                    desired_heading = self.headingAngle(
                        self.state, self.targets)
                    if self.distanceToGoal > self.minimumDistance:
                        self.targets[2] = desired_heading
                    else:
                        self.targets[2] = self.yaw_desired
                    # Increase the weight of the 3rd state (yaw) so that the robot heads towards the goal ASAP
                    self.weights[2] = 60.0

                # For computing the excution time of MPPI
                start = time.time()
                # Compute the optimal control
                u, normalizer, min_cost = self.jackal_controller.compute_control(
                    self.state,
                    [self.std_n, self.R, self.weights, self.targets])
                # The excution time of MPPI
                t_mppi = time.time() - start
                # Record the costs
                costs = self.robot.cost(self.state, u,
                                        [self.weights, self.targets, self.R])
                self.state_cost = costs[0]
                self.control_cost = costs[1]
                # Record and update the state, and the mppi excution time
                self.control_history.append(u)
                self.state_history.append(np.copy(self.state))
                self.desired_state_history.append(np.copy(self.targets))
                self.state_cost_history.append(self.state_cost)
                self.min_cost_history.append(min_cost)
                self.control_cost_history.append(self.control_cost)
                self.mppi_time_history.append(t_mppi)

                if self.print_out:
                    sys.stdout.write(
                        "Current States: (%f, %f, %f), t_mppi: %f \n" %
                        (self.state[0], self.state[1],
                         self.state[2] * 180 / np.pi, t_mppi))
                    sys.stdout.write("U: (%f, %f) \n" % (u[0], u[1]))
                    print(
                        "------------------------------------------------------"
                    )
                    sys.stdout.flush()
                if np.isnan(np.sum(self.state)):
                    print("Breaking MPPI due to numerical errors")
                    self.fail = True
                    break

                # Publish the linear and angular velocities to the robot
                self.cmd_vel_msg = Twist()
                self.cmd_vel_msg.linear.x = u[0]
                self.cmd_vel_msg.linear.y = 0
                self.cmd_vel_msg.linear.z = 0
                self.cmd_vel_msg.angular.x = 0
                self.cmd_vel_msg.angular.y = 0
                self.cmd_vel_msg.angular.z = u[1]
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # Publish the optimal predicted path obtained by MPPI
                self.publish_predicted_path()

                # Check if the goal pose is reached
                self.check_goal_reached(self.Jackal_states, self.targets)
                self.rate.sleep()
        except rospy.ROSInterruptException:
            print("ROS Terminated")
            pass

    """@brief: Plotting the states, corresponding optimal control action, and instantaneous running cost """
    def dataPlotting(self):
        uls.statePlotting(self.state_history, self.results_rootpath)
        uls.controlPlotting(self.control_history, self.results_rootpath)
        uls.costPlotting(self.state_cost_history, self.control_cost_history,
                         self.min_cost_history, self.mppi_time_history,
                         self.results_rootpath)

    ''' @brief: Retrieving the controllers' parameters, the costmap information, and summary of the performance'''
    def test_summary(self):
        if self.distanceToGoal > 0.5:
            self.local_minima = True
        else:
            self.local_minima = False
        #self.map_info = 'cost-map'
        control_history = np.array(self.control_history)
        if max(control_history[:, 0]) > self.max_linear_velocity or max(
                control_history[:, 1]) > self.max_angular_velocity:
            self.violate_ctrl_const = True
        else:
            self.violate_ctrl_const = False
        x, y = [s[0] for s in self.state_history
                ], [s[1] for s in self.state_history]
        self.pathLength = uls.pathLength(x, y)
        # Compute the avarage excution time of MPPI over all iterations
        self.av_t_mppi = np.mean(np.array(self.mppi_time_history)[20:])
        if self.av_t_mppi < 1 / float(self.hz):
            self.real_time_mppi = True
        else:
            self.real_time_mppi = False

        uls.testSummaryMPPI(
            self.init_pose, self.targets, self.max_linear_velocity,
            self.max_angular_velocity, self.map_info, self.time_horizon,
            self.hz, self.weights, self.num_trajectories, self.R,
            self.Sigma_du, self.exploration_variance, self.lambda_,
            self.SG_window, self.SG_PolyOrder, self.dist_type,
            self.local_minima, self.violate_ctrl_const, self.pathLength,
            self.av_t_mppi, self.real_time_mppi, self.results_rootpath)

        uls.save_results(self.state_history, self.desired_state_history,
                         self.state_cost_history, self.min_cost_history,
                         self.control_history, self.mppi_time_history,
                         self.results_rootpath)


if __name__ == "__main__":
    MPPI_Controller_Node = mppiControllerNode()
    MPPI_Controller_Node.run_mppi()
    MPPI_Controller_Node.test_summary()
    MPPI_Controller_Node.dataPlotting()
