
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import Joy
import std_srvs.srv

from datetime import datetime

import tf

import os
import math
import random
import numpy as np

from mission_file_parser import MissionFileParser

class MissionControl():

    """Class that takes a mission file and executes the mission"""

    def __init__(self):
        
        # Load a mission and parse the file
        mission_file = rospy.get_param('~mission_file')
        stage_simulation = rospy.get_param('~stage_simulation', default=True)
        if not os.path.exists(mission_file):
            rospy.logerr('Mission file not found: {}'.format(mission_file))
            exit()

        # Return the waypoints defined in the mission file    
        self.mission = MissionFileParser(mission_file).get_mission()

        self.mission_index = 0 # Currently executed command
        self.random_waypoint_number = 0 # Number of random waypoints remaining
        self.current_target = [0,0,0] # Current goal pose

        self.command_start = rospy.Time.now().to_sec() # Time when command execution started
        #self.command_timeout = rospy.get_param('~command_timeout', default=360.0)
        self.command_timeout = 1200 #1200
        self.recovery_timeout = self.command_timeout / 4
        self.recovery_start = rospy.Time.now().to_sec()
        self.positions = []
        self.query_time = rospy.Time.now().to_nsec()
        self.total_time = 0
        self.num_samples = 0

        self.costmap = None # Cache for the costmap

        self.tf_broadcaster = tf.TransformBroadcaster()

        # ROS topics
        self.start_pub = rospy.Publisher('/start', Empty, queue_size=1)
        self.stop_pub = rospy.Publisher('/stop', Empty, queue_size=1)
        self.abort_pub = rospy.Publisher('/abort', Empty, queue_size=1)
        self.target_pub = rospy.Publisher('relative_target', PoseStamped, queue_size=1)
        # self.cmd_pub = rospy.Publisher('mppi/cmd_vel', Twist, queue_size=1)

        self.costmap_sub = rospy.Subscriber('local_costmap', OccupancyGrid, \
                self.__costmap_callback__)
        self.costmap_update_sub = rospy.Subscriber('local_costmap_updates', \
                OccupancyGridUpdate, self.__costmap_update_callback__)
        self.joystick_sub = rospy.Subscriber('/joy', Joy, self.joystick_callback)

        # Connect to the action API of the move_base package
        self.navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while not self.navigation_client.wait_for_server(rospy.Duration(5)):
            rospy.loginfo('Waiting for mppi move_base action server')

        if stage_simulation:
          rospy.wait_for_service('/gazebo/reset_simulation')

            
        # Start the mission if it is not empty
        if len(self.mission) > 0:
            rospy.loginfo('Start mission')
            self.__send_next_command__()
        else:
            rospy.logerr('Mission file contains no commands')
            rospy.signal_shutdown('Mission Finished')

    def __send_next_command__(self):
        """
        Send the next command in the mission list
        """
        rospy.loginfo('sending next command')
        if len(self.mission) <= self.mission_index:
            rospy.loginfo('Mission Finished')
            rospy.signal_shutdown('Mission Finished')

        # Possible commands and their callback
        call = {'wp': self.__goto_waypoint__, \
                'cmd': self.__execute_command__, \
                'rd': self.__goto_random__ }

        item = self.mission[self.mission_index]
        call[item[0]](item[1])

    def __goto_waypoint__(self, coordinates):
        """
        Send the goal given by coordinates to the move_base node
        """
        rospy.loginfo('Go to waypoint: {}'.format(coordinates))

        # Generate an action message
        goal = MoveBaseAction()
        goal.action_goal.goal.target_pose.header.stamp = rospy.Time.now()

        goal.action_goal.goal.target_pose.header.frame_id = 'map'
        goal.action_goal.goal.target_pose.pose.position.x = coordinates[0]
        goal.action_goal.goal.target_pose.pose.position.y = coordinates[1]

        # Get the quaternion from the orientation in degree
        yaw = coordinates[2] * (math.pi/360.0)
        goal.action_goal.goal.target_pose.pose.orientation.z = math.sin(yaw)
        goal.action_goal.goal.target_pose.pose.orientation.w = math.cos(yaw)

        self.current_target = coordinates

        self.command_start = rospy.Time.now().to_sec()
        self.recovery_start = rospy.Time.now().to_sec() # Sams

        # Send the waypoint
        self.navigation_client.send_goal(goal.action_goal.goal, self.__done_callback__, \
                self.__active_callback__, self.__feedback_callback__)

    def __goto_random__(self, parameters):
        """
        Process the random waypoint command

        Sample a random waypoint from a uniform distribution and send it with the __goto_waypoint__
        function."""

        # \param "parameters" defines the number of waypoints N and the range in x and y direction, 
        # where parameters = [N, min_x, max_x, min_y, max_y].

        # first call: Set the number of random waypoints
        if self.random_waypoint_number == 0:
            self.random_waypoint_number = parameters[0]

        rospy.loginfo('Goto random waypoint: {} remaining'.format(self.random_waypoint_number))

        # Sample a valid goal pose
        found_valid_sample = False
        while not found_valid_sample:
            target = [0.0] * 3
            target[0] = random.uniform(parameters[1], parameters[2])
            target[1] = random.uniform(parameters[3], parameters[4])
            target[2] = random.uniform(0.0, 360.0)

            found_valid_sample = self.__check_target_validity__(target)

        self.__goto_waypoint__(target)

    def __feedback_callback__(self, feedback):
        """
        Callback for the feedback during the execution of __goto_waypoint__

        We compute the relative pose of the global target pose within the base frame and
        publish it as ROS topic
        """
        tot_time = rospy.Time.now().to_nsec() - self.query_time
        self.total_time = self.total_time + tot_time
        self.num_samples = self.num_samples + 1
        self.query_time = rospy.Time.now().to_nsec()
        # Check if we reached the timeout
        if (rospy.Time.now().to_sec() - self.command_start) >\
                self.command_timeout:
            rospy.loginfo('Timeout for command execution')

            self.navigation_client.cancel_goal()
            return

        # Compute the relative goal pose within the robot base frame
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()

        goal_position_difference = [self.current_target[0] - feedback.base_position.pose.position.x,
                                    self.current_target[1] - feedback.base_position.pose.position.y]
        # TODO position info
        pos = (feedback.base_position.pose.position.x, 
                feedback.base_position.pose.position.y,
                self.current_target[0],
                self.current_target[1])
        self.positions.append(pos)

        # Get the quaternion from the current goal
        yaw = self.current_target[2] * math.pi/ 180.0
        q = tf.transformations.quaternion_from_euler(0,0,yaw)

        current_orientation = feedback.base_position.pose.orientation
        p = [current_orientation.x, current_orientation.y, current_orientation.z, \
                current_orientation.w]

        # Rotate the relative goal position into the base frame
        goal_position_base_frame = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_inverse(p),
                tf.transformations.quaternion_multiply([goal_position_difference[0],
                    goal_position_difference[1], 0, 0], p))

        # Compute the difference to the goal orientation
        orientation_to_target = tf.transformations.quaternion_multiply(q, \
                tf.transformations.quaternion_inverse(p))
        target.pose.orientation.x = orientation_to_target[0]
        target.pose.orientation.y = orientation_to_target[1]
        target.pose.orientation.z = orientation_to_target[2]
        target.pose.orientation.w = orientation_to_target[3]

        target.pose.position.x = goal_position_base_frame[0]
        target.pose.position.y = -goal_position_base_frame[1]

        self.target_pub.publish(target)

        self.tf_broadcaster.sendTransform((self.current_target[0], self.current_target[1], 0),
                        q,
                        rospy.Time.now(),
                        'goal',
                        'map')

    def __done_callback__(self, state, result):
        """
        Callback when the execution of __goto_waypoint__ has finished

        We check if the execution was successful and trigger the next command
        """
        if state == GoalStatus.SUCCEEDED:
            # Publish stop message and reduce number of random waypoints
            rospy.loginfo('Reached waypoint')

            self.stop_pub.publish(Empty())

            # Sample was valid, so reduce count by one
            if self.random_waypoint_number > 0:
                self.random_waypoint_number -= 1

        else:
            # Execution was not successful, so abort execution and reset the simulation
            rospy.loginfo('Action returned: {}'.format(GoalStatus.to_string(state)))
            self.abort_pub.publish(Empty())
            self.__reset_simulation__()

            if self.random_waypoint_number > 0:
                rospy.loginfo('Resample this random waypoint')

        # Wait shortly before publishing the next command
        rospy.sleep(1)

        if self.random_waypoint_number > 0:
            # Their are still random waypoints to generate
            item = self.mission[self.mission_index]
            self.__goto_random__(item[1])
        else:
            # Previous command has finished, so go to the next command in the list
            self.mission_index += 1
            self.__send_next_command__()

    def __active_callback__(self):
        """
        Callback when the execution of __goto_waypoint__ starts

        We publish this event as ROS topic
        """
        self.start_pub.publish(Empty())

    def __execute_command__(self, cmd):
        """
        Execute an arbitrary command

        We currently only log it
        """
        rospy.loginfo('Execute command: {}'.format(cmd))

        self.command_start = rospy.Time.now()
        self.recovery_start = rospy.Time.now()

        self.mission_index += 1
        self.__send_next_command__()

    def __costmap_callback__(self, data):
        """
        Cache the given costmap
        """
        self.costmap = data
        # print(self.costmap)

    def __costmap_update_callback__(self, data):
        """
        Update the state of the cached costmap
        """
        if self.costmap:
            # Transform 1D array into 2D array
            update = np.array(data.data).reshape([data.height,data.width])
            current = np.array(self.costmap.data).reshape([self.costmap.info.height,
                self.costmap.info.width])

            # Replace the subsection within the costmap with the new data
            current[data.y:data.y+data.height,data.x:data.x+data.width] = update

            # Restore old 1D array
            self.costmap.data = current.flatten()
            
    def __check_target_validity__(self, target):
        """
        Check if the given target is located in an obstacle or its inflation radius
        """
        threshold = 10

        if self.costmap:
            # Compute the coordinates in the costmap
            x_pixel = int((target[0] - self.costmap.info.origin.position.x) /
                    self.costmap.info.resolution)
            y_pixel = int((target[1] - self.costmap.info.origin.position.y) /
                    self.costmap.info.resolution)

            return self.costmap.data[int(x_pixel + self.costmap.info.width * y_pixel)] < threshold
        else:
            rospy.logwarn('No costmap available')
            return True

    def __reset_simulation__(self):
        """
        Reset the simulation by calling the ROS service
        """
        try:
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', std_srvs.srv.Empty)
            reset_simulation()
        except rospy.ServiceException as e:
            print('Service call failed: {}'.format(e))
            
    def joystick_callback(self, data):
        # Abort planning 
        if data.buttons[4] == 1 and data.buttons[5] == 1:
            rospy.signal_shutdown('Mission Finished')


