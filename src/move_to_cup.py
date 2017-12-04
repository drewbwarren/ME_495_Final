#!/usr/bin/env python

import rospy
import sys
import moveit_commander
from transforms import so3_to_quat
import numpy as np
import geometry_msgs.msg
from baxter_core_msgs.srv import SolvePositionIK
import baxter_interface
import sensor_msgs.msg

class move_arm(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')

        # Test out the pre planned pose of an object - the tube
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.stamp = rospy.Time.now()
        pose_target.header.frame_id = 'base'
        R = np.array([[0,0,1],[0,-1,0],[1,0,0]])
        quat = so3_to_quat(R)
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3]
        pose_target.pose.position.x = 1.1
        pose_target.pose.position.y = 0.0
        pose_target.pose.position.z = 0.0

        # Test the pose from the example on the page http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service
        # pose_target = geometry_msgs.msg.PoseStamped()
        # pose_target.header.stamp = rospy.Time.now()
        # pose_target.header.frame_id = 'base'
        # pose_target.pose.orientation.x = -0.366894936773
        # pose_target.pose.orientation.y = 0.885980397775
        # pose_target.pose.orientation.z = 0.108155782462
        # pose_target.pose.orientation.w = 0.262162481772
        # pose_target.pose.position.x = 0.657579481614
        # pose_target.pose.position.y = 0.851981417433
        # pose_target.pose.position.z = 0.0388352386502

        self.ik_solve([pose_target])

    def ik_solve(self,pose): # pose is a PoseStamped
        # ik_srv = SolvePositionIK()

        limb = baxter_interface.Limb('left')
        # limb.move_to_neutral()
        angles = limb.joint_angles()
        # rospy.loginfo(angles)

        # Create the joint_states message for the seed
        jt_state = sensor_msgs.msg.JointState()
        jt_state.header.stamp = rospy.Time.now()
        jt_state.header.frame_id = 'base'
        jt_state.name = limb.joint_names()
        jt_state.name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
        jt_state.position = (limb.joint_angles()).values()
        # Guess for the example from online
        # jt_state.position = [0.4371845240478516, 1.8419274289489747, 0.4981602602966309, -1.3483691110107423, -0.11850001572875977, 1.18768462366333, -0.002300971179199219]



        ########### Attempt with known pose and angles
        jt_state.name = ['left_w0','left_w1','left_w2','left_e0','left_e1','left_s0','left_s1']
        jt_state.position = [0.36418045875370275,-0.7089378719821386,-0.9498846200989162,-2.357752019787287,1.3610064374902215,-0.5741206949450195,0.8185350153221069]
        pose[0].pose.position.x = 1.2 #1.0879208264873947
        pose[0].pose.position.y = .1 #0.05337168831627051
        pose[0].pose.position.z = .1 #0.2391984213113991
        pose[0].pose.orientation.x = 0.7548514360043578
        pose[0].pose.orientation.y = 0.014655696409408715
        pose[0].pose.orientation.z = 0.6533744959938046
        pose[0].pose.orientation.w = 0.055554370752947196





        rospy.loginfo('ik solve time!')
        ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
        # result = ik_srv(pose,[jt_state],1)
        result = ik_srv(pose,[],0)
        rospy.loginfo(result)#.joints[0])

        angles = {}
        i = 0
        for name in result.joints[0].name:
            angles[name] = result.joints[0].position[i]
            i = i + 1
        print angles

        # For resetting
        # angles = {'left_w0': 0, 'left_w1': 0, 'left_w2': 0, 'left_e0': 0, 'left_e1': 0, 'left_s0': 0, 'left_s1': 0}
        # angles = {'left_w0': -2.6482788962422887, 'left_w1': 0.6693933481254781, 'left_w2': 3.1415-2.8139783215524545, 'left_e0': -0.9159500402361473, 'left_e1': 1.7026987496906347, 'left_s0': -0.3153010662297495, 'left_s1': -0.43638282693640296}
        # limb.move_to_joint_positions(angles)

        if result.isValid[0]:
            self.create_plan(angles)


    def create_plan(self,angles):
        self.group.clear_pose_targets()
        # self.group.set_start_state(
        self.group.set_joint_value_target(angles)
        plan = self.group.plan()
        self.group.execute(plan)



if __name__ == '__main__':
    rospy.init_node('grasp')
    move_arm()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
