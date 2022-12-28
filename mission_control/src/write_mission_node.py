#!/usr/bin/env python2

import rospy
from write_mission import WriteMission

def main():

    rospy.init_node('write_mission')

    write_mission = WriteMission()

    rospy.spin()

if __name__ == "__main__":
    main()