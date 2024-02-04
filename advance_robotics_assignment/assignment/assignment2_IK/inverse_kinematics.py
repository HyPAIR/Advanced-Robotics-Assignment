import os
import sys
import numpy as np
import rospy
import tf
path_ws = os.path.abspath('../../..') 
sys.path.append(path_ws)
sys.path.append(path_ws + '/advance_robotics_assignment/')
sys.path.append(path_ws + '/advance_robotics_assignment/franka_ros_interface')

from franka_controller.interfaces import ArmController
from solution.solveIK import IK
from solution.transformation_utils import transformation

rospy.init_node("ik_visualization")

# --- Use your code to implement IK class in solveIK.py ---
ik = IK()

# --- Use your code to implement transformation class in transformation_utils.py---
transform = transformation() 

# Rviz communication
tf_broad  = tf.TransformBroadcaster()

# Broadcasts the transform from given frame to world frame
def show_pose(H,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(H),
        tf.transformations.quaternion_from_matrix(H),
        rospy.Time.now(),
        frame,
        "world"
    )

# Required target points are provided in the assignment description document.
# Note: Use "transformation" class in transformation_utils.py to generate the target points. 
targets = []

np.set_printoptions(suppress=True)

# Execution
if __name__ == "__main__":

    arm = ArmController()

    # Iterates through the given targets, using your IK solution.
    for i, target in enumerate(targets):
        show_pose(target,"target")

        # Use the initial position of the robotic arm as an initial guess
        initial_guess = arm.neutral_position() 

        # Use your IK solver in solveIK.py
        q, success = ik.inverse(target, initial_guess)

        if success:
            arm.move_to_position(q)

        if i < len(targets) - 1:
            input("Press Enter to move to next target...")