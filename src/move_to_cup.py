#!/usr/bin/env python

import rospy
import sys
import moveit_commander

from transforms import so3_to_quat
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class grasping(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')
        self.group.set_goal_position_tolerance(0.01)

        ################ Collision Object
        table = CollisionObject()
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.stamp = rospy.Time.now()
        box_pose.header.frame_id = 'tablelol'
        box_pose.pose.position.x = 1.25
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0
        table.primitives.append(primitive)
        table.primitive_poses.append(box_pose)
        table.operation = table.ADD
        self.scene.add_box('table',box_pose,size=(.077,.073,.122))

        ### Subscribe to the cup pose
        rospy.Subscriber('/cup', geometry_msgs.msg.PoseStamped(), cupcb)


    def ik_solve(self,pose):
        jt_state = sensor_msgs.msg.JointState()
        jt_state.header.stamp = rospy.Time.now()
        jt_state.header.frame_id = 'base'
        jt_state.name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.707
        pose.pose.orientation.w = 0.0

        ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
        i = 0
        while true:
            result = ik_srv([pose],[jt_state],0)
            i += 1
            if result.isValid[0] or i==4:
                raise Exception('No IK solution found.')
                break

        angles = {}
        i = 0
        for name in result.joints[0].name:
            angles[name] = result.joints[0].position[i]
            i = i + 1

        return angles

    def create_plan(self,angles):
        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        self.group.set_joint_value_target(angles)
        plan = self.group.plan()
        self.group.execute(plan)

    def cupcb(self,pose):
        angles = self.ik_solve(pose)
        create_plan(angles)

if __name__ == '__main__':
    rospy.init_node('grasp')
    grasping()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
