# ME_495_Final

### Members: 
##### Suhail Sulaiman, Drew Warren, Andrew Wentzel, Zidong (Tom) Xiao, Kaiyang (Kevin) Zheng

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

#### Connect to Baxter.  A walthrough for that is avalible on the [rethink robotics website](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup). A good primer on troubleshooting and available resources can be found on the [MSR website](http://nu-msr.github.io/embedded-course-site/notes/16_baxter_introduction.html).

#### [roslaunch me_495_final grab.launch](https://github.com/tehwentzel/ME_495_Final/blob/master/launch/grab.launch) will launch the whole configureation

#### [moveit_start.launch](https://github.com/tehwentzel/ME_495_Final/blob/master/launch/baxter_moveit_config.launch) will launch just the moveit setup for the robot, as well as the arm_mover.py node.  This file relies on having the baxter_moveit_config package installed to configure moveit to run with baxter. For more info, a tutorial for running moveit with baxter can be found (here)[http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial]

### Explaination of Main Nodes

#### move_to_cup
* Takes in the pose of the cup, runs that through Baxter's built in IK service, and the creates and executes a path plan to that pose using MoveIt!. So far, the MoveIt! planner for pose targets is not working, so this node solves the IK for the joints and puts that joint target into the MoveIt! planner for joint targets. It's still a little finicky and needs more development to make it more robust, especially calling the IK service and creating the path plan.

#### grasp.py
* is a very simple node to control the joint positions of Baxter's left arm. Starting from rest, the arm moves to three intermediate positions before moving to a plausible grasping position above the table. A better approach would be to use the `moveit_commander` to plan a path to the position of the cup and avoid the known obstacle of the table. I will try to do that next.

#### ALSO OTHER NODES USED TO PICK UP THE CUP

#### filter_V2
* This node reads the messages sent to the /skeletons topic, which holds the locations of the skeletons found by the skeletontracker_nu package.  It will filter the skeletons rank them based on how close they are and their angle off the center of the camera.  Once a main user is identified, it publishes the xyz values of the right hand to the /target_poses topics as a Floast32MultiArray

#### move_arm.py
* This node is used to run the moveit path planning to follow the position of the hand of the user.
* The node first waits for an std_msgs/Bool message to be sent to the /cup_grabbed topic to signify that the cup is grabbed.  Once this happens, a [callback](https://github.com/tehwentzel/ME_495_Final/blob/386c071c6f99d2dd3017174a3459c69b87a42177/src/move_arm.py#L276) will instantiate a subscriber to the /target_poses topics. and unsubscribe to the /cup_grabbed topic.  The FilterV2.py node will publish a Float32MultiArray message to /target_poses with the x,y,z coordiantes of the user's right hand int he Asus Xtion's frame.  The move_arm node will transform the point into baxter's planning frame and use Moveit to move the arm to that point.  This node will also look at the input into the hands depth sensor, and when it stops seeing the cup (which acts as a signal that the cup has been taken), it will move to a neutral position and turn off.

        #### Additions from Drew

        `get_joint_states.py` is an even simpler node that retreives the joint states of only the left arm and prints them to the screen. Used for debugging. I'm adding it because it may be useful to others for debugging, but we might want to put it in the `.gitignore` before the end.

        We have a "working" system for grasping the cup. To grab the cup, follow these steps:
        1. In one terminal, launch [`baxter_moveit_config.launch`][config]. The launches the move_group and planning_context nodes, and somewhere in there it uploads Baxter's SRDF.
        2. In another terminal, run the [`track_cup.py`][track] node. This node does the OpenCV to find the center of a red cup.
        3. In another terminal, run the [`scan.py`][scan] node. This node places the arm in a good position to see what's on the table, and it pans the hand until it is pointing at the cup, then it stops. All joint movements are done using `baxter_interface`, not MoveIt!.
        4. Kill the `scan.py` node and run the joint trajectory action server node with `rosrun baxter_interface joint_trajectory_action_server.py`. This action server is needed for MoveIt to be able to control joints. If you want to use `baxter_interface` to move joints, you will have to kill this node.
        5. In another terminal, run the [`move_to_cup.py`][move] node. Using MoveIt!, this node creates a path for the hand to move directly forward a fixed distance, and then close the gripper. A big improvement to be made is to sense if/when the cup is in the grippers before stopping the motion or before gripping. The current idea is for the hand to move forward along this path until the range sensor senses the cup, and then create a new path to that position.

        Another main improvement that needs to be made is streamlining this process. Instead of needing to start each node manually, let's figure out how to do it all at once. This will involve launch files for some of it. One challenge is figuring out how to start and stop the joint trajectory action server when needed. I will be working on that. Another challenge is to figure out how to signal from one node to another when a task is finished and when the next task should start.

        [config]:https://github.com/tehwentzel/ME_495_Final/blob/grasp/launch/baxter_moveit_config.launch
        [track]:https://github.com/tehwentzel/ME_495_Final/blob/grasp/src/track_cup.py
        [scan]:https://github.com/tehwentzel/ME_495_Final/blob/grasp/src/scan.py
        [move]:https://github.com/tehwentzel/ME_495_Final/blob/grasp/src/move_to_cup.py

### More Resources

* Anyone interested in constrained planning in moveit might be interested in looking into [using OMPL create an Approximation Manifold For Path Planning](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/constraints_approximation_database.html).  THis allows you to calculate and approximated sub-space of valid states to plan on for a set of constraints that is created when the node is run.  Requires using C++

* An ongoing area of development in moveit is integrating [CHOMP](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/chomp_interface_tutorial.html) with Moveit.  CHOMP Uses optimization based path planning.  Using alternative path planning algorithms that allow a cost function based on the distance offset from the desired pose at any point would likely prove a better alternative to naive path planning or the default method of constrained planning used in Moveit.
### Video Demo
Please refer to this [video](https://drive.google.com/drive/folders/1px5vzHt1ewg4ES5YicfgGZbBv6ElgH1i) for the demo of this project
