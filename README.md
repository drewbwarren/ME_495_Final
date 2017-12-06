# ME_495_Final

### Installations

#### Follow the instructions to install moveit! here: [moveit! for Baxter](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)
* For kinetic users, the command is: sudo apt-get install ros-kinetic-moveit

#### Install a bug fix for Moveit
* Run: sudo apt-get install python-pyassimp

#### [trac_ik](https://bitbucket.org/traclabs/trac_ik.git): Library used for the Trac-IK Plugin for moveit
* Run the command: sudo apt-get install ros-kinetic-trac-ik
* Replace the kinematics.yaml file with the one found in ME_495_Final/config/kinematics.yaml
* As a default, kinematics.yaml is located at ~/($WORKSPACE NAME)/src/moveit_robots/baxter/baxter_moveit_config/config

### Additions from Drew

`grasp.py` is a very simple node to control the joint positions of Baxter's left arm. Starting from rest, the arm moves to three intermediate positions before moving to a plausible grasping position above the table. A better approach would be to use the `moveit_commander` to plan a path to the position of the cup and avoid the known obstacle of the table. I will try to do that next.

`get_joint_states.py` is an even simpler node that retreives the joint states of only the left arm and prints them to the screen. Used for debugging. I'm adding it because it may be useful to others for debugging, but we might want to put it in the `.gitignore` before the end.

`move_to_cup.py` takes in the pose of the cup, runs that through Baxter's built in IK service, and the creates and executes a path plan to that pose using MoveIt!. So far, the MoveIt! planner for pose targets is not working, so this node solves the IK for the joints and puts that joint target into the MoveIt! planner for joint targets. It's still a little finicky and needs more development to make it more robust, especially calling the IK service and creating the path plan.

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


[NxR skeleton tracker msg](https://github.com/NxRLab/skeletonmsgs_nu)
[NxR skeleton tracker lib](https://github.com/NxRLab/skeletontracker_nu)
[Asus Xtion Driver](https://answers.ros.org/question/109411/asus-xtion-problems-with-ubuntu-1204-running-ros-fuerte/)
