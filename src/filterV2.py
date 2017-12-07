#!/usr/bin/env python
################
# ROS IMPORTS: #
################
import rospy
import tf
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
####################
# NON ROS IMPORTS: #
####################
import math
import copy
from math import fmod, pi, copysign
import numpy as np

####################
# GLOBAL VARIABLES #
####################


# frequency divider for checking "key" skeleton:
FREQ_DIV = 30
ANG_MULT = 20
DIST_MULT = 1

class SkeletonController:
    def __init__(self):
        self.skel_sub = rospy.Subscriber("skeletons", Skeletons,
                                         self.skelcb)
        self.count= 0
        self.running_flag = False
        self.pos_pub = rospy.Publisher('target_poses',Float32MultiArray,queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        

    def skelcb(self, data):
        if len(data.skeletons) == 0:
            return
        if self.count%FREQ_DIV == 0:
            self.get_key_user(data.skeletons)
        self.count += 1
        if self.key_index < len(data.skeletons) and \
                data.skeletons[self.key_index].userid == self.key_id:
            skel = data.skeletons[self.key_index]
        else:
            for i,skel in enumerate(data.skeletons):
                if skel.userid == self.key_id:
                    found = True
                    break
                found = False
            if not found:
                rospy.logwarn("Could not find a skeleton userid that matches the key user")
                return
        if self.running_flag:
            for i,con in enumerate(self.controllers):
                jtrans = skel.__getattribute__(con.joint).transform.translation
                # tracker mirrors, so flip x
                jpos = [-1.0*jtrans.x, jtrans.y, jtrans.z, 1]


        self.x = skel.right_hand.transform.translation.x
        self.y = skel.right_hand.transform.translation.y
        self.z = skel.right_hand.transform.translation.z
        trans = Float32MultiArray()
        trans.data = [self.x,self.y,self.z]
        self.pos_pub.publish(trans)
        rospy.loginfo(trans)
       # rospy.sleep(0.5)

    def get_key_user(self, skels):
        data = []
        for i,s in enumerate(skels):
            v2 = np.array([s.head.transform.translation.x,
                           s.head.transform.translation.z])
            ang = np.arccos(v2[1]/np.linalg.norm(v2))
            dist = v2[1]
            cost = ANG_MULT*ang + DIST_MULT*dist
            data.append([i, s.userid, cost])
        val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
        self.key_index = data[idx][0]
        self.key_id = data[idx][1]
        return


    def data():

        [x,y,z] = skelcb()

        return [x,y,z]




def main():
    rospy.init_node('skeleton_filter')#, log_level=rospy.INFO)
   
    # rate = rospy.Rate(1)
    
    # while not rospy.is_shutdown():
    #     sim = SkeletonController()
    #     rate.sleep()

    try:
         sim = SkeletonController()
         x = sim.x
    except rospy.ROSInterruptException: pass

    rospy.spin()

    


if __name__=='__main__':
    main()