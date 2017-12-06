#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import roslaunch

from transforms import so3_to_quat
import geometry_msgs.msg
import sensor_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

from baxter_core_msgs.srv import SolvePositionIK
import baxter_interface

import tf2_ros
import tf2_geometry_msgs

class grasping(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('left_arm')
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_max_velocity_scaling_factor(.2)

        # IK Service
        self.ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)

        # gripper and limb init
        self.gripper = baxter_interface.Gripper('left')
        self.gripper.open()
        self.limb = baxter_interface.Limb('left')

        #################### TF stuff
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        #################### Cup Subscriber
        rospy.Subscriber('cup_center',geometry_msgs.msg.Point,self.cupcb)
        self.cup_plan = True

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

        # start joint trajectory action server
        # node = roslaunch.core.Node('baxter_interface','joint_trajectory_action_server.py','rsdk_position_w_id_joint_trajectory_action_server',output='screen')
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # process = launch.launch(node)
        # print process.is_alive()
        # rospy.sleep(5)

        self.wait_for_cup()

        self.grab_pub = rospy.Publisher('cup_grabbed',Bool,queue_size=10)
        self.grab_pub.publish(False)

    def wait_for_cup(self):
        rospy.Subscriber('cup_found',Bool,self.cup_status_cb)
        def hook():
            self.wait = False
        rospy.on_shutdown(hook)
        self.wait = True
        while self.wait:
            pass

    def cup_status_cb(self,cup_status):
        if cup_status:
            self.wait = False

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
        while True:
            result = self.ik_srv([pose],[seed],0)
            i += 1
            if i==4:
                raise Exception('No IK solution found.')
                break
            elif result.isValid[0]:
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

    def forward_to_cup(self,z):
        jt_state = sensor_msgs.msg.JointState()
        jt_state.header.stamp = rospy.Time.now()

        angles = self.limb.joint_angles()
        jt_state.name = list(angles.keys())
        jt_state.position = list(angles.values())
        jt_state.header.frame_id = 'base'

        cup_pose = geometry_msgs.msg.PoseStamped()
        cup_pose.header.frame_id = 'left_gripper'
        cup_pose.header.stamp = rospy.Time.now()
        cup_pose.pose.position.x = 0.0
        cup_pose.pose.position.y = 0.0
        cup_pose.pose.position.z = z
        cup_pose.pose.orientation.x = 0 #0.2733787988108373
        cup_pose.pose.orientation.y = 0# 0.6514094419398767
        cup_pose.pose.orientation.z = 0 #0.22283432444334322
        cup_pose.pose.orientation.w = 1 #0.6717697783930155
        transform = self.tf_buffer.lookup_transform('base',
                                       cup_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second
        pose_transformed = tf2_geometry_msgs.do_transform_pose(cup_pose, transform)

        angles = self.ik_solve(pose_transformed,jt_state)
        self.create_plan(angles)
        # self.gripper.close()
        # self.grab_pub.publish(True)

    def cupcb(self,point):
        rospy.loginfo(point.z)
        # if point.z > 0.05 and self.cup_plan:
        #     self.group.stop()
        #     rospy.loginfo('\n\nnew plan\n\n')
        #     self.forward_to_cup(point.z)
        #     rospy.loginfo(point.z)
        #     self.cup_plan = False
        if point.z > 0.0 and point.z < .08:
            rospy.loginfo('STOP')
            self.group.stop()
            self.gripper.close()
            self.grab_pub.publish(True)

if __name__ == '__main__':
    rospy.init_node('grasp')

    t = grasping()
    t.forward_to_cup(.45)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
