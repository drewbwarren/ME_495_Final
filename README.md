# ME_495_Final

### Additions from Drew

`grasp.py` is a very simple node to control the joint positions of Baxter's left arm. Starting from rest, the arm moves to three intermediate positions before moving to a plausible grasping position above the table. A better approach would be to use the `moveit_commander` to plan a path to the position of the cup and avoid the known obstacle of the table. I will try to do that next.

`get_joint_states.py` is an even simpler node that retreives the joint states of only the left arm and prints them to the screen. Used for debugging. I'm adding it because it may be useful to others for debugging, but we might want to put it in the `.gitignore` before the end.

`move_to_cup.py` takes in the pose of the cup, runs that through Baxter's built in IK service, and the creates and executes a path plan to that pose using MoveIt!. So far, the MoveIt! planner for pose targets is not working, so this node solves the IK for the joints and puts that joint target into the MoveIt! planner for joint targets. It's still a little finicky and needs more development to make it more robust, especially calling the IK service and creating the path plan.

