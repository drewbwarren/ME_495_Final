#!/usr/bin/env python

import rospy
import baxter_interface




def main():

    rospy.init_node('grasp_cup')

    limb = baxter_interface.Limb('left')
    gripper = baxter_interface.Gripper('left')

    dt = 1.5

    # print the current joint angles
    angles = limb.joint_angles()
    print "start"
    gripper.open()

    # set to resting position
    print "Resting position"
    angles = {'left_w0': -0.1906941248266376, 'left_w1': 0.026716761540192202, \
         'left_w2': -0.01660067902654383, 'left_e0': -0.016297876392492583, \
         'left_e1': 0.49439022848238245, 'left_s0': 0.19248096014046112, \
         'left_s1': 1.0470000705645015}
    limb.move_to_joint_positions(angles)
    rospy.sleep(dt)


    # Intermediate positions
    print "Intermediate position 1"
    angles = {'left_w0': -1.4047525741372482, 'left_w1': -0.8636733195223281, 'left_w2': 2.659161182238135, 'left_e0': -0.4802762026648031, 'left_e1': 1.896288365627596, 'left_s0': 1.155651446790003, 'left_s1': 0.7190049021071392}
    limb.move_to_joint_positions(angles)
    rospy.sleep(dt)

    print "Intermediate position 2"
    angles['left_e0'] = -2.0167412942606475
    angles['left_e1'] = 2.2966410763775826
    angles['left_s0'] = -0.35517381661962943
    angles['left_s1'] = 0.6455736786437072
    angles['left_w0'] = -1.4863372978500582
    angles['left_w1'] = -1.5708000245836633
    angles['left_w2'] = 1.0444053202201324
    limb.move_to_joint_positions(angles)
    rospy.sleep(dt)


    # table position
    print "grasping position"
    angles = {'left_w0': -2.6482788962422887, 'left_w1': 0.6693933481254781, 'left_w2': -2.8139783215524545, 'left_e0': -0.9159500402361473, 'left_e1': 1.7026987496906347, 'left_s0': -0.3153010662297495, 'left_s1': -0.43638282693640296}
    limb.move_to_joint_positions(angles)
    rospy.sleep(dt)

    gripper.close()


if __name__=='__main__':
    main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
