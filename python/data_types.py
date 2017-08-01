import numpy as np
trial_type=np.dtype([('reached_pose','f8',6)])
ilqr_type=np.dtype([('K','f8',(100,16,16)),('k_ff','f8',(100,6)),('x','f8',(100,16)),('u','f8',(100,16))])

trajectory_type=np.dtype([('method',np.str_,16),("solution_pose",'f8',6),("joint_trajectory",'f8',(100,16)),("object_trajectory",'f8',(100,6)),('trial_data_open_loop',trial_type,5),('ilqr',ilqr_type,1)])
grasp_type = np.dtype([ ('joint_angles','f8',16),('contact_fingers','i',4), ('rigid_finger', 'i',4),('rigid_transform','f8',6),('initial_pose','f8',6),('desired_pose','f8',6),('optimization',trajectory_type,1)])


