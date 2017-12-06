#!/usr/bin/env python
import rospy
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint
import math
from math import fmod, pi, copysign
import numpy as np

#global variables 

#define angular and distance weights 
FREQ_DIV = 30
ANG_MULT = 20
DIST_MULT = 1

#call back function 
def skelcb(data):

	key_index, key_id = get_key_user(data.skeletons)
	skel = data.skeletons[key_index]
	headZ = skel.head.transform.translation
	rospy.loginfo(headZ)




#track the most important one
def get_key_user(skeletons):
        data = []
        for i,s in enumerate(skeletons):
            v2 = np.array([s.head.transform.translation.x,
                           s.head.transform.translation.z])
            ang = np.arccos(v2[1]/np.linalg.norm(v2))
            dist = v2[1]
            cost = ANG_MULT*ang + DIST_MULT*dist
            data.append([i, s.userid, cost])
        val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
        key_index = data[idx][0]
        key_id = data[idx][1]
        #rospy.loginfo(data)
        return key_index, key_id




def main():
	rospy.init_node('skeleton_filter')
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		filter= rospy.Subscriber("skeletons",Skeletons,skelcb)
		rate.sleep()

if __name__ == '__main__':
	main()
