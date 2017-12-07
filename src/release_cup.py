#!/usr/bin/env python


import argparse

import rospy

import baxter_interface
import baxter_external_devices
from baxter_core_msgs.msg import EndpointState

from baxter_interface import CHECK_VERSION

def get_present_state(data,left):
    y = data.wrench.force.y
    if y<-2:
        left.open()
        gripper_pub.publish(True)




def main():

    rospy.init_node("release_cup")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    left = baxter_interface.Gripper('left', CHECK_VERSION)
    # right = baxter_interface.Gripper('right', CHECK_VERSION)
    rs.enable()
    rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, get_present_state,left, queue_size=1)
    gripper_pub = rospy.Publisher('cup_taken',Bool,queue_size=10)
    gripper_pub.publish(False)

    done=False
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            if c=='w':
                left.open()
            elif c=='q':
                left.close()




if __name__ == '__main__':
    main()
