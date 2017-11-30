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
        R = np.array([[-1,0,0],[0,1,0],[0,0,1]])
        quat = so3_to_quat(R)
        pose_target.pose.orientation.x = quat[0]
        pose_target.pose.orientation.y = quat[1]
        pose_target.pose.orientation.z = quat[2]
        pose_target.pose.orientation.w = quat[3]
        pose_target.pose.position.x = 1.1
        pose_target.pose.position.y = 0.0
        pose_target.pose.position.z = 1.014998

        # Test the pose from the example on the page http://sdk.rethinkrobotics.com/wiki/API_Reference#Inverse_Kinematics_Solver_Service
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.stamp = rospy.Time.now()
        pose_target.header.frame_id = 'base'
        pose_target.pose.orientation.x = -0.366894936773
        pose_target.pose.orientation.y = 0.885980397775
        pose_target.pose.orientation.z = 0.108155782462
        pose_target.pose.orientation.w = 0.262162481772
        pose_target.pose.position.x = 0.657579481614
        pose_target.pose.position.y = 0.851981417433
        pose_target.pose.position.z = 0.0388352386502

        self.ik_solve([pose_target])

    def ik_solve(self,pose): # pose is a PoseStamped
        # ik_srv = SolvePositionIK()

        limb = baxter_interface.Limb('left')
        angles = limb.joint_angles()
        # rospy.loginfo(angles)

        # Create the joint_states message for the seed
        jt_state = sensor_msgs.msg.JointState()
        jt_state.header.stamp = rospy.Time.now()
        jt_state.header.frame_id = 'base'
        jt_state.name = limb.joint_names()
        jt_state.name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
        jt_state.position = (limb.joint_angles()).values()
        jt_state.position = [0.4371845240478516, 1.8419274289489747, 0.4981602602966309, -1.3483691110107423, -0.11850001572875977, 1.18768462366333, -0.002300971179199219]
        # jt_state.velocity = limb.joint_velocities()
        # jt_state.effort = limb.joint_efforts()

        rospy.loginfo('ik solve time!')
        ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
        result = ik_srv(pose,[jt_state],0)
        # result = ik_srv(pose,[],0)
        rospy.loginfo(result.joints[0])

        angles = {}
        i = 0
        for name in result.joints[0].name:
            angles[name] = result.joints[0].position[i]

        self.create_plan(angles)


    def create_plan(self,angles):
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
