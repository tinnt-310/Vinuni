# Immutable, fully validated command handed from the planning worker to the
# ros2_control executor. It is replaced atomically as one generation.
uint8 JOINT=0
uint8 FREE_SPACE=1
uint8 CARTESIAN_LINEAR=2

std_msgs/Header header
uint64 generation
uint8 mode
trajectory_msgs/JointTrajectory trajectory
geometry_msgs/Pose path_start
geometry_msgs/Pose path_goal
float64 planned_duration_s
uint64 planning_scene_revision
