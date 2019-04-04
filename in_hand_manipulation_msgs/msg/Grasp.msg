# This message type stores information about a grasp, used for gait planning
# 
# obj_name= object currently grasped.
# joint_state= joint configuration of the robot in current grasp.
# f_index= Fingertips in contact with the object.(index)
# contact_pts= contact points between the object and the fingertips in the palm frame of the robot.
# contact_normals= contact normals at the contact surface.
# obj_pose= Pose of the object in the palm frame.
# ellipses= approximate 2D ellipses of contact regions for fingertips in palm frame.
# reachable_ws_mesh= file locations for meshes of the fingertip workspace(stl).
# obj_mesh= file location of object mesh(stl).
#

std_msgs/String obj_name
sensor_msgs/JointState joint_state
std_msgs/Int64[] f_index
geometry_msgs/PointStamped[] contact_pts
geometry_msgs/Vector3[] contact_normals
geometry_msgs/PoseStamped obj_pose
in_hand_manipulation_msgs/Ellipse[] ellipses
std_msgs/String[] reachable_ws_mesh
std_msgs/String obj_mesh