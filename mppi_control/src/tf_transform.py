#!/usr/bin/env python3
"""
@author: VAIL, IU
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf

def handle_jackal_pose(msg):
    br = tf.TransformBroadcaster()
    t = TransformStamped()
    # t.header.stamp = rospy.Time.now()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation = msg.pose.pose.position
    t.transform.rotation = msg.pose.pose.orientation
    br.sendTransformMessage(t)

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    rospy.Subscriber("/ground_truth/odom", Odometry, handle_jackal_pose)
    rospy.spin()
