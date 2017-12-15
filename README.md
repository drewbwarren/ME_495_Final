
ME495 Embedded Systems in Robotics: Baxter Butler 
==============
#### *Group 2: Suhail Sulaiman, Drew Warren, Andrew Wentzel, Zidong (Tom) Xiao, Kaiyang (Kevin) Zheng*

##  Overview of Project
The objective of our project was to have Baxter locate and handle containers having liquid in it. At first Baxter will sweep the table to locate the container using the camera in the limb and the IR sensor. Once the container is located, baxter will grab it and then start tracking the right hand of the person standing in front of him. Then baxter will keep following the right hand of the person with the container until the cup is handed over to the user. Once the user pulls the cup baxter will sense the pull and release the gripper and the hand will go back to home position 
A short video can be found: *[Here](https://www.youtube.com/watch?v=RMCaAgLhMFE&feature=youtu.be)*

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/RMCaAgLhMFE/0.jpg)](http://www.youtube.com/watch?v=RMCaAgLhMFE)

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/v5kqGhG_XYQ/0.jpg)](http://www.youtube.com/watch?v=v5kqGhG_XYQ)

### Installations

#### Follow the instructions to install moveit! here: [moveit! for Baxter](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)
* For kinetic users, the command is: sudo apt-get install ros-kinetic-moveit

#### Install a bug fix for Moveit
* Run: sudo apt-get install python-pyassimp

#### [trac_ik](https://bitbucket.org/traclabs/trac_ik.git): Library used for the Trac-IK Plugin for moveit
* Run the command: sudo apt-get install ros-kinetic-trac-ik
* Replace the kinematics.yaml file with the one found in ME_495_Final/config/kinematics.yaml
* As a default, kinematics.yaml is located at ~/($WORKSPACE NAME)/src/moveit_robots/baxter/baxter_moveit_config/config

#### Install the [skeletontracker_nu package](https://github.com/NxRLab/skeletontracker_nu) and the corresponding [Skeletonmsgs_nu](https://github.com/NxRLab/skeletonmsgs_nu) packages from git.  These are used to gather to implement skeleton tracking with the Kinect

#### Install the Drivers for the Kinect: See [this post](https://answers.ros.org/question/109411/asus-xtion-problems-with-ubuntu-1204-running-ros-fuerte/), and look at the top response by Jarvis where he deals with the issue.

### Launching the Package

* Connect to Baxter.  A walthrough for that is avalible on the [rethink robotics website](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup). A good primer on troubleshooting and available resources can be found on the [MSR website](http://nu-msr.github.io/embedded-course-site/notes/16_baxter_introduction.html).

* [nu_skeletontracker.launch](https://github.com/NxRLab/skeletontracker_nu/blob/indigo/launch/nu_skeletontracker.launch) lauches the skeletontracker. It is recommended to launch the skeletontracker and the whole configuration seperately, since we found the Asus Xtion crashes more often when these two launch files are launched at the same time.

* [roslaunch me_495_final grab.launch](https://github.com/tehwentzel/ME_495_Final/blob/master/launch/grab.launch) will launch the whole configureation

* [moveit_start.launch](https://github.com/tehwentzel/ME_495_Final/blob/master/launch/baxter_moveit_config.launch) will launch just the moveit setup for the robot, as well as the arm_mover.py node.  This file relies on having the baxter_moveit_config package installed to configure moveit to run with baxter. For more info, a tutorial for running moveit with baxter can be found (here)[http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial]

### Main Nodes
#### [track_cup](https://github.com/tehwentzel/ME-495_Final/blob/master/src/track_cup.py)
* This node uses open cv to detect red objects in the camera frame and publishes the centroid of the shape as x and y coordinates on to `cup_center` topic. This node also uses the IR sensor to measure the depth and puclishes it as th z coordinate.

#### [scan](https://github.com/tehwentzel/ME-495_Final/blob/master/src/scan.py)
* This node places the arm in a position where the wrist and end effector are parallel to the table and the camera can see the surface. Joint left_w0 rotates slowly so that the camera can look at everything on the table. When the cup moves to the middle of the camera's view according to the x and y pixel positions from the track_cup node, the joint stops moving, and the node signals to the move_to_cup node to start its task.

#### [move_to_cup](https://github.com/tehwentzel/ME-495_Final/blob/master/src/move_to_cup.py)
* Takes in the pose of the cup, runs that through Baxter's built in IK service, and the creates and executes a path plan to that pose using MoveIt!. So far, the MoveIt! planner for pose targets is not working, so this node solves the IK for the joints and puts that joint target into the MoveIt! planner for joint targets. It's still a little finicky and needs more development to make it more robust, especially calling the IK service and creating the path plan.

#### [filter_V2](https://github.com/tehwentzel/ME-495_Final/blob/master/src/filter_V2.py)
* This node reads the messages sent to the /skeletons topic, which holds the locations of the skeletons found by the skeletontracker_nu package.  It will filter the skeletons rank them based on how close they are and their angle off the center of the camera.  Once a main user is identified, it publishes the xyz values of the right hand to the /target_poses topics as a Floast32MultiArray

#### [move_arm.py](https://github.com/tehwentzel/ME-495_Final/blob/master/src/move_arm.py)
* This node is used to run the moveit path planning to follow the position of the hand of the user.
* The node first waits for an std_msgs/Bool message to be sent to the /cup_grabbed topic to signify that the cup is grabbed.  Once this happens, a [callback](https://github.com/tehwentzel/ME_495_Final/blob/386c071c6f99d2dd3017174a3459c69b87a42177/src/move_arm.py#L276) will instantiate a subscriber to the /target_poses topics. and unsubscribe to the /cup_grabbed topic.  The FilterV2.py node will publish a Float32MultiArray message to /target_poses with the x,y,z coordiantes of the user's right hand int he Asus Xtion's frame.  The move_arm node will transform the point into baxter's planning frame and use Moveit to move the arm to that point.  This node will also look at the input into the hands depth sensor, and when it stops seeing the cup (which acts as a signal that the cup has been taken), it will move to a neutral position and turn off.

#### [release_cup.py](https://github.com/tehwentzel/ME-495_Final/blob/master/src/release_cup.py)
* This node senses if the user is pulling the cup and open the gripper if a pull is sensed. As the robot is always made to move with the cup vertical, the force in the y direction of the end effector will never be negative and if it goes negative it will be due to a pull

### More Resources

* Anyone interested in constrained planning in moveit might be interested in looking into [using OMPL create an Approximation Manifold For Path Planning](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/constraints_approximation_database.html).  THis allows you to calculate and approximated sub-space of valid states to plan on for a set of constraints that is created when the node is run.  Requires using C++

* An ongoing area of development in moveit is integrating [CHOMP](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/chomp_interface_tutorial.html) with Moveit.  CHOMP Uses optimization based path planning.  Using alternative path planning algorithms that allow a cost function based on the distance offset from the desired pose at any point would likely prove a better alternative to naive path planning or the default method of constrained planning used in Moveit.
