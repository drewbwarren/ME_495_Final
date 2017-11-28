#!/usr/bin/env python

import rospy
import baxter_interface

def main():
    rospy.init_node('grasp_cup')

    limb = baxter_interface.Limb('left')

    # print the current joint angles
    angles = limb.joint_angles()
    print angles

if __name__ == '__main__':
    main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
