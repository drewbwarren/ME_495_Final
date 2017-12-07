#!/usr/bin/env python

import rospy
import baxter_interface
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


### Orientation of scanning pose
# 'orientation': Quaternion(x=0.2733787988108373, y=0.6514094419398767, z=-0.22283432444334322, w=0.6717697783930155)}

class scanner(object):

    def __init__(self):
        self.limb = baxter_interface.Limb('left')
        self.gripper = baxter_interface.Gripper('left')
        # self.limb.move_to_neutral()
        # start_angles = {'left_w0': 1.048092373322709, 'left_w1': -1.571179821991635, 'left_w2': 0.06941263065181497, 'left_e0': 0.005752427954570301, 'left_e1': 1.45076233014263, 'left_s0': 0.18829614171293454, 'left_s1': 0.10085923680346595}
        # start_angles = {'left_w0': 1.535514768673299, 'left_w1': -1.5270778743399294, 'left_w2': 0.0222427214243385, 'left_e0': 0.006135923151541655, 'left_e1': 1.5358982638702705, 'left_s0': 0.5798447378206864, 'left_s1': 0.07784952498518474}
        start_angles = {'left_w0': 1.57, 'left_w1': -1.57, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 1.57, 'left_s0': 0.21475731030395792, 'left_s1': 0.06212622190935926}
        start_angles = {'left_w0': 1.0711020851409903, 'left_w1': -1.5707963267946636, 'left_w2': -0.014956312681882784, 'left_e0': 0.041033986075934815, 'left_e1': 1.4948642777943357, 'left_s0': 0.21475731030395792, 'left_s1': 0.06212622190935926}
        start_angles = {'left_w0': 1.1286263646866932, 'left_w1': -1.570029336400721, 'left_w2': -0.05714078434873166, 'left_e0': 0.0395000052880494, 'left_e1': 1.522475931976273, 'left_s0': 0.17103885784922362, 'left_s1': 0.02147573103039579}


        # position 1
        # start_angles = {'left_w0': 0.5039126888203584, 'left_w1': -1.5707963267946636, 'left_w2': -0.35473305719850196, 'left_e0': 0.24351945007680942, 'left_e1': 1.220665211959818, 'left_s0': 0.019941750242510378, 'left_s1': 0.18561167533413506}

        # Position2
        # start_angles = {'left_w0': -2.316694484903946, 'left_w1': -1.3510535789300782, 'left_w2': -0.11619904468232009, 'left_e0': 0.17640779060682257, 'left_e1': 1.5209419511883877, 'left_s0': -1.4595827196729712, 'left_s1': 0.18484468494019235}
        # start_angles = {'left_w0': -1.7418351846438873, 'left_w1': -1.565810889234036, 'left_w2': -0.02454369260616662, 'left_e0': -0.0011504855909140602, 'left_e1': 1.4189322287940078, 'left_s0': -1.3211409535663126, 'left_s1': 0.13115535736420286}



        self.limb.move_to_joint_positions(start_angles)
        rospy.Subscriber('cup_center',Point,self.cup_cb)
        self.cup = False
        self.gripper.open()
        rospy.loginfo('gripper open')

        self.end_pub = rospy.Publisher('cup_found',Bool,queue_size=10)
        self.end_pub.publish(False)

    def scan(self):
        while not self.cup:
            def hook():
                self.cup = True
            scan_vel = {'left_w0': 0.1, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}
            self.limb.set_joint_velocities(scan_vel)
            rospy.on_shutdown(hook)

    def cup_cb(self, point):
        # rospy.loginfo("cup found")
        # rospy.loginfo(point.x)
        if point.x > 640 and point.x < 677:
            self.cup = True
            # rospy.loginfo('\n\ncup found\n\n')
            self.end_pub.publish(True)


if __name__=='__main__':
    rospy.init_node('scan')
    myScanner = scanner()
    myScanner.scan()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        self.cup = True
