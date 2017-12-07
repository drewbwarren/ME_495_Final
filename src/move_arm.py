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

from moveit_commander import MoveGroupCommander

#translations from the world frame to the left shoulder and sonar ring (or .03 above the sonar ring)
lls = [0.064, 0.259, 0.130]
sr = [0.095, 0, 0.82]
start_pose = [.9, 0.2 ,0.130]

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
        #self.group.set_planner_id("LBKPIECEkConfigDefault")
        #this node will scale any tf pose requests to be at most max_reach from the base frame
        self.max_reach = rospy.get_param('~max_reach', default=.7)
        #define a start pose that we can move to before stuff runs
        self.start_pose = PoseStamped()
        self.start_pose = self.get_start_pose()
        #remove this when working for realz
        self.display_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=10)
        self.rate = rospy.Rate(1)

    def callback(self, targetarray):
        #callback that moves in a constrained path to anything published to /target_poses
        ##First, scale the position to be withing self.max_reach
        #new_target = self.project_point(targetarray.data)
        new_target = self.project_point(targetarray)
        target = Pose()
        target.position = new_target
        #change orientation to be upright
        target.orientation = self.start_pose.pose.orientation
        #clear group info and set it again
        self.group.clear_pose_targets()
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(10)
        self.group.set_pose_target(target)
        #plan and execute plan. If I find a way, I should add error checking her
        #currently, if the plan fails, it just doesn't move and waits for another pose to be published
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.spin()
        return

    def callback_tester(self, targetarray):
        #callback that moves in a constrained path to anything published to /target_poses
        ##First, scale the position to be withing self.max_reach
        #new_target = self.project_point(targetarray.data)
        targetarray2 = targetarray
        targetarray3 = targetarray
        targetarray2.data = (targetarray.data[0]+.1,targetarray.data[1]+.1,targetarray.data[2]+.1)
        targetarray3.data = (targetarray.data[0]+.15,targetarray.data[1]-.1,targetarray.data[2]-.1)
        new_target = self.project_point(targetarray)
        new_target2 = self.project_point(targetarray2)
        new_target3 = self.project_point(targetarray3)
        target = Pose()
        target2 = Pose()
        target3 = Pose()
        target.position = new_target
        target2.position = new_target2
        target3.position = new_target3
        #change orientation to be upright
        target.orientation = self.start_pose.pose.orientation
        #clear group info and set it again
        self.group.clear_pose_targets()
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(10)
        self.group.set_pose_target(target)
        #plan and execute plan. If I find a way, I should add error checking her
        #currently, if the plan fails, it just doesn't move and waits for another pose to be published
        plan = self.group.plan()
        self.group.set_pose_target(target2)
        self.group.clear_path_constraints()
        self.group.set_path_constraints(self.get_constraint())
        plan2 = self.group.plan()
        self.group.clear_path_constraints()
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_pose_target(target3)
        plan3 = self.group.plan()
        self.group.execute(plan)
        self.move_start()
        self.group.execute(plan2)
        self.move_start()
        self.group.execute(plan3)
        self.move_start()
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
        #I copied this from an example but have no idea what it does
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._left_joint_names = self._left_arm.joint_names()
        print(self._left_arm.endpoint_pose())
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 100Hz
        self._pub_rate.publish(100)
        return

    def set_neutral(self):
        """
       Sets both arms back into a neutral pose.
       """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        print('Is neutral now')
        return

    def get_start_pose(self,point=start_pose,rpy=[0, math.pi/2, 0]):
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
        x = multiarray.data[2] + sr[0] - start_pose[0]
        y = multiarray.data[0] + sr[1] - start_pose[1]
        z = (-1*multiarray.data[1]) + sr[2] - start_pose[2]
        #scale point to a finite reach distance from the shoulder
        obj_dist = math.sqrt(x**2 + y**2 + z**2)
        scale_val = min(self.max_reach/obj_dist,.99)
        point_scaled = Point()
        #scale point and bring into the base frames
        point_scaled.x = scale_val*x + start_pose[0]
        point_scaled.y = scale_val*y + start_pose[1]
        point_scaled.z = scale_val*z + start_pose[2]
        return(point_scaled)

    def move_random(self):
        #moves baxter to a random position.  used for testing
        randstate = PoseStamped()
        randstate = self.group.get_random_pose()
        self.group.clear_pose_targets()
        self.group.set_pose_target(randstate)
        self.group.set_planning_time(8)
        self.scale_movegroup()
        plan = self.group.plan()
        while len(plan.joint_trajectory.points) == 1 and not rospy.is_shutdown():
            print('plan no work')
            plan = self.group.plan()
        self.group.execute(plan)
        self.rate.sleep()
        return

    def move_random_constrained(self):
        #move baxter to a random position with constrained path planning.  also for testing
        self.scale_movegroup()
        randstate = PoseStamped()
​
271
            mover.move_start()        randstate = self.group.get_random_pose()
        self.group.clear_pose_targets()
        self.group.set_pose_target(randstate)
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(100)
        self.scale_movegroup()
        constrained_plan = self.group.plan()
        self.group.execute(constrained_plan)
        self.unscale_movegroup()
        rospy.sleep(3)
        return

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

    def get_constraint(self, euler_orientation = [0,math.pi/2,0], tol = [3,3,.3]):
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

def cup_callback(grabbedness):
    has_run = 0
    if grabbedness and not has_run:
        #slows down the robot path plan        
        mover.scale_movegroup()
        mover.move_start()
        rospy.Subscriber('target_poses', Float32MultiArray, mover.callback,queue_size=1)
        has_run = 1;


if __name__ == '__main__':
    try:
        mover = MoveCup()
        while not rospy.is_shutdown():
            #enables the robot
            #mover.start_baxter_interface()
            #moves the robot to a starting pose that makes future moves fail less
            #mover.set_neutral()

            #mover.move_start()
            #sets up the subscriber for the callback, currently set to take a pose
            rospy.Subscriber('cup_grabbed', Bool, cup_callback,queue_size=1)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
