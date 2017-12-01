#!/usr/bin/env python

# https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/scripts/move_group_python_interface_tutorial.py

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

## BEGIN_SUB_TUTORIAL imports
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import baxter_interface
import tf
## END_SUB_TUTORIAL

from std_msgs.msg import (Header, String, UInt16)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_msgs.msg import (Constraints, OrientationConstraint)
from baxter_interface import CHECK_VERSION

from moveit_commander import MoveGroupCommander

class MoveCup():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_tutorial')
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        #use joint_group parameter to change which arm it uses?
        joint_group = rospy.get_param('~arm', default="left_arm")
        self.group = MoveGroupCommander(joint_group)
        self.p = PoseStamped()
        self.p.header.frame_id = self.robot.get_planning_frame()
        #remove this when working for realz
        self.display_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=10)

    def scale_movegroup(self,vel = .4,acc = .9):
        self.group.set_max_velocity_scaling_factor(vel)
        self.group.set_max_acceleration_scaling_factor(acc)

    def unscale_movegroup(self):
        self.group.set_max_velocity_scaling_factor(0)
        self.group.set_max_acceleration_scaling_factor(0)

    def start_baxter_interface(self):
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

    def set_p(self,x=.05,y=0,z=0,ox=0,oy=0,oz=0,w=1):
        new_p = PoseStamped()
        new_p.header.frame_id = self.robot.get_planning_frame()
        new_p.p.pose.position.x = x
        new_p.p.pose.position.y = y
        new_p.p.pose.position.z = z

        new_p.p.pose.orientation.x = ox
        new_p.p.pose.orientation.y = oy
        new_p.p.pose.orientation.z = oz
        new_p.p.pose.orientation.w = w
        self.p = new_p
        rospy.spin()
        return(new_p)

    def get_end_pose(self):
        end_pose_dict = self._left_arm.endpoint_pose()
        end_pose = Pose()
        end_pose.position = end_pose_dict['position']
        end_pose.orientation = end_pose_dict['orientation']
        print('end pose:')
        print(end_pose)
        return(end_pose)

    def set_group_start_to_current_pose(self):
        self.group.set_start_state(self.get_end_pose())

    def set_target(self,x=-.5,y=-.5,z=.6,ox=.6,oy=.4,oz=0,w=0):
        target = PoseStamped()
        target.header.frame_id = self.robot.get_planning_frame()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        target.pose.orientation.x = ox
        target.pose.orientation.y = oy
        target.pose.orientation.z = oz
        target.pose.orientation.w = w
        self.group.set_pose_target(target)
        return(target)

    def move_random(self):
        self.set_group_start_to_current_pose()
        randstate = PoseStamped()
        randstate = self.group.get_random_pose()
        self.group.clear_pose_targets()
        self.group.set_pose_target(randstate)
        self.group.set_planning_time(10)
        self.scale_movegroup()
        self.group.plan()
        self.group.go()
        rospy.spin()
        return

    def move_random_constrained(self):
        self.set_group_start_to_current_pose()
        randstate = PoseStamped()
        randstate = self.group.get_random_pose()
        self.group.clear_pose_targets()
        self.group.set_pose_target(randstate)
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(100)
        self.scale_movegroup()
        constrained_plan = self.group.plan()
        self.group.execute(constrained_plan)
        rospy.spin()
        return

    def move_target(self):
        self.group.clear_pose_targets()
        self.set_group_start_to_current_pose()
        self.set_target()
        self.group.set_planning_time(15)
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.spin()
        return

    def move_target_constrained(self):
        self.group.clear_pose_targets()
        self.set_group_start_to_current_pose()
        self.set_target()
        self.group.set_path_constraints(self.get_constraint())
        self.group.set_planning_time(50)
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.spin()
        return

    def get_constraint(self, euler_orientation = [0,.5,0], tol = [1,1,1]):
        q_orientation = tf.transformations.quaternion_from_euler(euler_orientation[0],euler_orientation[1],euler_orientation[2])
        orientation_msg = Quaternion(q_orientation[0],q_orientation[1],q_orientation[2],q_orientation[3])

        constraint = Constraints()
        constraint.name = 'upright_wrist'

        upright_orientation = OrientationConstraint()
        upright_orientation.link_name = self.group.get_end_effector_link()
        upright_orientation.orientation = orientation_msg
        upright_orientation.absolute_x_axis_tolerance = tol[0]
        upright_orientation.absolute_y_axis_tolerance = tol[1]
        upright_orientation.absolute_z_axis_tolerance = tol[2]
        upright_orientation.weight = 1.0
        upright_orientation.header = self.p.header
        constraint.orientation_constraints.append(upright_orientation)
        return(constraint)



if __name__ == '__main__':
    try:
        mover = MoveCup()
        while not rospy.is_shutdown():
            mover.start_baxter_interface()
            mover.set_neutral()
            mover.move_target_constrained()
            rospy.sleep(10)
    except rospy.ROSInterruptException:
        pass