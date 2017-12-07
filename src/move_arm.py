#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import tf
import math
## END_SUB_TUTORIAL

from std_msgs.msg import (Header, String, UInt16, Float32MultiArray, Bool)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion, PointStamped)
from moveit_msgs.msg import (Constraints, OrientationConstraint, CollisionObject)
from shape_msgs.msg import SolidPrimitive
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import JointState

from moveit_commander import MoveGroupCommander
from baxter_core_msgs.srv import SolvePositionIK


#translations from the world frame to the left shoulder and sonar ring (or .03 above the sonar ring)
lls = [0.064, 0.259, 0.130]
sr = [0.095, 0, 0.82]

flag = True

class MoveCup():

    def __init__(self):
        #basic initiatioon
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_tutorial')
        self.robot = moveit_commander.RobotCommander()
        ################ Collision Object
        self.scene = moveit_commander.PlanningSceneInterface()
        table = CollisionObject()
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.stamp = rospy.Time.now()
        box_pose.header.frame_id = 'tablelol'
        box_pose.pose.position.x = 1.25
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = -0.6
        table.primitives.append(primitive)
        table.primitive_poses.append(box_pose)
        table.operation = table.ADD
        self.scene.add_box('table',box_pose,size=(.077,.073,.122))
        #use joint_group parameter to change which arm it uses?
        self.joint_group = rospy.get_param('~arm', default="left_arm")
        self.group = MoveGroupCommander(self.joint_group)
        #self.group.set_planner_id("BKPIECEkConfigDefault")
        #this node will scale any tf pose requests to be at most max_reach from the base frame
        self.max_reach = rospy.get_param('~max_reach', default=1.1)
        #define a start pose that we can move to before stuff runs
        self.start_pose = PoseStamped()
        self.start_pose = self.get_start_pose()
        #remove this when working for realz
        self.display_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        self.rate = rospy.Rate(1)
        self.ik_srv = rospy.ServiceProxy('ExternalTools/left/PositionKinematicsNode/IKService', SolvePositionIK)
        self.limb = baxter_interface.Limb('left')

    def callback(self, targetarray):
        #callback that moves in a constrained path to anything published to /target_poses
        ##First, scale the position to be withing self.max_reach
        #new_target = self.project_point(targetarray.data)
        new_target = self.project_point(targetarray)
        target = PoseStamped()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = 'base'
        target.pose.position = new_target
        #change orientation to be upright
        target.pose.orientation = self.start_pose.pose.orientation
        #clear group info and set it again
        self.group.clear_pose_targets()
        # self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(10)
        # self.group.set_pose_target(target)

        ################### Try joint space planning
        jt_state = JointState()
        jt_state.header.stamp = rospy.Time.now()
        angles = self.limb.joint_angles()
        jt_state.name = list(angles.keys())
        jt_state.position = list(angles.values())
        jt_state.header.frame_id = 'base'
        result = self.ik_srv([target],[jt_state],0)
        angles = {}
        i = 0
        for name in result.joints[0].name:
            angles[name] = result.joints[0].position[i]
            i = i + 1
        self.group.set_joint_value_target(angles)

        #plan and execute plan. If I find a way, I should add error checking her
        #currently, if the plan fails, it just doesn't move and waits for another pose to be published
        plan = self.group.plan()
        self.group.execute(plan)
        self.rate.sleep()
        return

    def scale_movegroup(self,vel = .9,acc = .9):
        #slows down baxters arm so we stop getting all those velocity limit errors
        self.group.set_max_velocity_scaling_factor(vel)
        self.group.set_max_acceleration_scaling_factor(acc)

    def unscale_movegroup(self):
        self.group.set_max_velocity_scaling_factor(1)
        self.group.set_max_acceleration_scaling_factor(1)

    def start_baxter_interface(self):
        #can enable the robot when running this node alone
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        return

    def get_start_pose(self,point=[.9, 0.2, 0],rpy=[0, math.pi/2, 0]):
        #define a starting position for the move_start method
        new_p = PoseStamped()
        new_p.header.frame_id = self.robot.get_planning_frame()
        new_p.pose.position.x = point[0]
        new_p.pose.position.y = point[1]
        new_p.pose.position.z = point[2]

        p_orient = tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
        p_orient = Quaternion(*p_orient)
        new_p.pose.orientation = p_orient
        return(new_p)

    def project_point(self,multiarray):
        #scales an array and returns a point (see: Pose.position) to be within self.max_reach
        #convert points from sonar ring frame to shoulder frame
        x = multiarray.data[2] + sr[0] - lls[0]
        y = multiarray.data[0] + sr[1] - lls[1]
        z = (-1*multiarray.data[1]) + sr[2] - lls[2]
        #scale point to a finite reach distance from the shoulder
        obj_dist = math.sqrt(x**2 + y**2 + z**2)
        scale_val = min(self.max_reach/obj_dist,.99)
        point_scaled = Point()
        #scale point and bring into the base frames
        point_scaled.x = scale_val*x + lls[0]
        point_scaled.y = scale_val*y + lls[1]
        point_scaled.z = scale_val*z + lls[2]
        return(point_scaled)

    def move_start(self):
        #move baxter to the self.start_pose pose
        self.group.clear_pose_targets()
        self.group.set_pose_target(self.start_pose)
        self.group.set_planning_time(10)
        print('moving to start')
        plan = self.group.plan()
        self.group.execute(plan)
        print('at start')
        self.rate.sleep()
        return

    def get_constraint(self, euler_orientation = [0,math.pi/2,0], tol = [3,3,.5]):
        #method takes euler-angle inputs, this converts it to a quaternion
        q_orientation = tf.transformations.quaternion_from_euler(euler_orientation[0],euler_orientation[1],euler_orientation[2])
        orientation_msg = Quaternion(q_orientation[0],q_orientation[1],q_orientation[2],q_orientation[3])
        #defines a constraint that sets the end-effector so a cup in it's hand will be upright, or straight upside-down
        constraint = Constraints()
        constraint.name = 'upright_wrist'
        upright_orientation = OrientationConstraint()
        upright_orientation.link_name = self.group.get_end_effector_link()
        upright_orientation.orientation = orientation_msg
        upright_orientation.absolute_x_axis_tolerance = tol[0]
        upright_orientation.absolute_y_axis_tolerance = tol[1]
        upright_orientation.absolute_z_axis_tolerance = tol[2]
        upright_orientation.weight = 1.0
        upright_orientation.header = self.start_pose.header
        constraint.orientation_constraints.append(upright_orientation)
        return(constraint)

    def init_sub(self):
        self.target_sub = rospy.Subscriber('target_poses', Float32MultiArray, self.callback,queue_size=1)

    def done(self):
        self.gripper = baxter_interface.Gripper('left')
        self.gripper.open()
        self.move_start()
        return

def cup_callback(grabbedness):
    if grabbedness and flag:
        #slows down the robot path plan
        Flag = False
        rospy.loginfo('ready for orientation plan')      
        mover.scale_movegroup()
        mover.move_start()
        mover.init_sub()
        sub.unregister()
        return

def release_callback(dummy):
    release_sub.unregister()
    mover.target_sub.unregister()
    mover.done()
    #rospy.spin()

if __name__ == '__main__':
    try:
        mover = MoveCup()
        while not rospy.is_shutdown():
            #enables the robot
            mover.start_baxter_interface()
            #moves the robot to a starting pose that makes future moves fail less
            #mover.set_neutral()

            #mover.move_start()
            #sets up the subscriber for the callback, currently set to take a pose
            sub = rospy.Subscriber('cup_grabbed', Bool, cup_callback,queue_size=1)
            release_sub = rospy.Subscriber('cup_taken', Bool, release_callback, queue_size = 1)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
