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
        self.group.set_max_velocity_scaling_factor(.3)

        # IK Service
        self.ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)

        # gripper init
        self.gripper = baxter_interface.Gripper('left')
        self.gripper.open()

        #################### TF stuff
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #################### Cup Subscriber
        rospy.Subscriber('cup_center',geometry_msgs.msg.Point,self.cupcb)

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



    def ik_solve(self,pose,seed):
        # jt_state = sensor_msgs.msg.JointState()
        # jt_state.header.stamp = rospy.Time.now()
        # jt_state.header.frame_id = 'base'
        # jt_state.name = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
        # pose.pose.orientation.x = 0.707
        # pose.pose.orientation.y = 0.0
        # pose.pose.orientation.z = 0.707
        # pose.pose.orientation.w = 0.0


        i = 0
        while true:
            result = self.ik_srv([pose],[seed],0)
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

    def forward_to_cup(self):
        jt_state = sensor_msgs.msg.JointState()
        jt_state.header.stamp = rospy.Time.now()

        angles = limb.joint_angles()
        jt_state.name = list(angles.keys())
        jt_state.position = list(angles.values())
        jt_state.header.frame_id = 'base'

        cup_pose = geometry_msgs.msg.PoseStamped()
        cup_pose.header.frame_id = 'left_gripper'
        cup_pose.header.stamp = rospy.Time.now()
        cup_pose.pose.position.x = 0.0
        cup_pose.pose.position.y = 0.0
        cup_pose.pose.position.z = .3
        cup_pose.pose.orientation.x = 0 #0.2733787988108373
        cup_pose.pose.orientation.y = 0# 0.6514094419398767
        cup_pose.pose.orientation.z = 0 #0.22283432444334322
        cup_pose.pose.orientation.w = 1 #0.6717697783930155
        transform = self.tf_buffer.lookup_transform('base',
                                       cup_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
        pose_transformed = tf2_geometry_msgs.do_transform_pose(cup_pose, transform)

        angles = self.ik_solve([pose_transformed],[jt_state])
        self.create_plan(angles)
        self.gripper.close()

    def cupcb(self,pose):
        pass

if __name__ == '__main__':
    rospy.init_node('grasp')
    grasping()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
