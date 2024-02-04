import os
import sys
import rospy
import tf
import geometry_msgs.msg
path_ws = os.path.abspath('../../..') 
sys.path.append(path_ws)
sys.path.append(path_ws + '/advance_robotics_assignment/')
sys.path.append(path_ws + '/advance_robotics_assignment/franka_ros_interface')
from franka_controller.interfaces import ArmController
from solution.solveFK import FK
rospy.init_node("fk_visualization")

# --- Use your code to implement FK class in solveFK.py ---
fk = FK()

#  Rviz publisher 
tf_broad  = tf.TransformBroadcaster()
point_pubs = [
    rospy.Publisher('/vis/joint' + str(i), geometry_msgs.msg.PointStamped, queue_size=10)
    for i in range(7)
]

# Broadcasts the transform from given frame to world frame
def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e),
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "world"
    )

# Visualize the end effector
def show_all_FK(state):
    q = state['position']
    joints, T0e = fk.forward(q)
    show_pose(T0e,"endeffector")

# Define a set of configurations below matches the dimensions given in the assignment.
# Note: each configuration is 1 * 7
configurations = []

if __name__ == "__main__":
    # Visualization
    arm = ArmController(on_state_callback=show_all_FK)
    for i, q in enumerate(configurations):
        arm.move_to_position(q)
        if i < len(configurations) - 1:
            input("Press Enter to move to next configuration...")
        else:
            input("All configurations are complete!")
    arm.move_to_position(q)