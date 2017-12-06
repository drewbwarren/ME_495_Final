#!/usr/bin/env python

import rospy
import baxter_interface

def main():
    rospy.init_node('grasp_cup')

    limb = baxter_interface.Limb('left')

    # print the current joint angles
    angles = limb.joint_angles()
    print angles

    # print the endpoint pose
    pose = limb.endpoint_pose()
    print pose

if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
