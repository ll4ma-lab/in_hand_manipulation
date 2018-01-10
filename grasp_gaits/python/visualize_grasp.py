from viz_markers import VizInHand
import dataset_class
from geometry_msgs.msg import PoseStamped
import PyKDL

if __name__=='__main__':
    g_n=10
    #
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    d=dataset_class.datasetRead()
    mesh=d.read_obj_file('Banana')
    print mesh
    pose,cpts,_=d.read_grasp('Banana',g_n)
    R=PyKDL.Rotation()
    R=R.RPY(pose[3],pose[4],pose[5])
    pose[3],pose[4],pose[5],pose[6]=R.GetQuaternion()
    obj_pose=PoseStamped()
    obj_pose.pose.position.x=pose[0]
    obj_pose.pose.position.y=pose[1]
    obj_pose.pose.position.z=pose[2]
    obj_pose.pose.orientation.x=pose[3]
    obj_pose.pose.orientation.y=pose[4]
    obj_pose.pose.orientation.z=pose[5]
    obj_pose.pose.orientation.w=pose[6]


    obj_mesh='../data/object_models/banana/meshes/poisson_mesh.stl'
    o_mesh='file://'+grasp_loc+'/data/object_models/banana_bin.stl'
    f_meshes=[]
    viz=VizInHand(o_mesh,f_meshes)
    viz.viz_obj_mesh(obj_pose,a=1.0,rgb=[1.0,1.0,0.0])
    viz.viz_pts(cpts)
