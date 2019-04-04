#
import rospy
from in_hand_manipulation_msgs.srv import UpdateObj
from geometry_msgs.msg import Pose
# load object from file into a pointcloud

# send to env server

if __name__=='__main__':
    obj_mesh='/home/bala/object_models/mustard_bottle/google_16k/nontextured.binvox'
    # call environment update node:
    rospy.wait_for_service('/update_obj')
    print "found service"
    
    update_env_cloud=rospy.ServiceProxy('/update_obj',UpdateObj)
    o_pose=Pose()
    o_pose.position.x=0.1
    o_pose.orientation.w=1.0
    
    res=update_env_cloud(o_pose,obj_mesh)
    #print res.init_env

