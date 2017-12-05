#!/usr/bin/env python

import rospy
import baxter_interface
from geometry_msgs.msg import Point


### Orientation of scanning pose
# 'orientation': Quaternion(x=0.2733787988108373, y=0.6514094419398767, z=-0.22283432444334322, w=0.6717697783930155)}

class scanner(object):

    def __init__(self):
        self.limb = baxter_interface.Limb('left')
        # self.limb.move_to_neutral()
        start_angles = {'left_w0': 1.048092373322709, 'left_w1': -1.571179821991635, 'left_w2': 0.06941263065181497, 'left_e0': 0.005752427954570301, 'left_e1': 1.45076233014263, 'left_s0': 0.18829614171293454, 'left_s1': 0.10085923680346595}
        self.limb.move_to_joint_positions(start_angles)
        rospy.Subscriber('cup_center',Point,self.cup_cb)
        self.cup = False

    def scan(self):
        while not rospy.is_shutdown():
            while not self.cup:
                scan_vel = {'left_w0': 0.1, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}
                self.limb.set_joint_velocities(scan_vel)

    def cup_cb(self, point):
        rospy.loginfo("cup found")
        rospy.loginfo(point)
        if point.x > 325 and point.x < 375:
            self.cup = True


if __name__=='__main__':
    rospy.init_node('scan')
    myScanner = scanner()
    myScanner.scan()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        self.cup = True
