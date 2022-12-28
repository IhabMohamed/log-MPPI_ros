#!/usr/bin/env python3

import rospy
from mission_control import MissionControl

def main():

    rospy.init_node('mission_control')

    mission = MissionControl()

    rospy.spin()

if __name__ == "__main__":
    main()
