import os
import rospy
import tf
import rospkg
import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from datetime import *


class WriteMission():
  
  def __init__(self):
    """
    Drive with joystick and select targets for autonomous mission.
    Requires robot to be localized.
    """
    print("Press green (A) button to add a target to the list or red (B) button to delete the last recorded target from the list.")
    storage_path = rospy.get_param('~storage_path', default=rospkg.RosPack().get_path('mission_control') + "/missions/")
    date_str = datetime.strftime(datetime.now(), '%Y-%m-%d_%H-%M-%S')
    self.storage_file = os.path.join(storage_path, date_str) + '.txt'
    self.transform_listener = tf.TransformListener()
    self.joystick_sub = rospy.Subscriber('/joy', Joy, self.__save_position_callback__)
    self.target_poses = []    
    
  def __save_targets_to_file__(self):
    with open(self.storage_file, 'w') as file:
      for pose in self.target_poses:
        euler_angles = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, 
                                                                 pose.pose.orientation.y, 
                                                                 pose.pose.orientation.z, 
                                                                 pose.pose.orientation.w])
        heading_degree = euler_angles[2]*180.0 / math.pi
        file.write('wp: {0} {1} {2}\n'.format(pose.pose.position.x, pose.pose.position.y, heading_degree))
  
  def __remove_last_target_from_file(self):
    os.system("sed -i '$ d' " + self.storage_file)
    
  
  def __save_position_callback__(self, data):
    if data.buttons[0] == 1:
      (base_position,base_orientation) = self.transform_listener.lookupTransform('/map', '/base_link', rospy.Time())
      current_pose = PoseStamped()
      current_pose.pose.position.x = base_position[0]
      current_pose.pose.position.y = base_position[1]
      current_pose.pose.position.z = base_position[2]
      current_pose.pose.orientation.x = base_orientation[0]
      current_pose.pose.orientation.y = base_orientation[1]
      current_pose.pose.orientation.z = base_orientation[2]
      current_pose.pose.orientation.w = base_orientation[3]
      self.target_poses.append(current_pose)
      rospy.loginfo('Saving pose with (x,y,phi)=({0}, {1}, {2})'.format(base_position[0], base_position[1], 
                                                                tf.transformations.euler_from_quaternion(base_orientation)[2]))
      rospy.loginfo('Target stack now has size {0}'.format(len(self.target_poses)))
      self.__save_targets_to_file__()
      
    elif data.buttons[1] == 1:
      rospy.loginfo('Removing latest target')
      if len(self.target_poses) >= 1:
        self.target_poses.pop()
        self.__remove_last_target_from_file()
      rospy.loginfo('Target stack now has size {0}'.format(len(self.target_poses)))
  
      
      
      
    