#
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
# import required allegro model files
from rospkg import RosPack
rp=RosPack()
rp.list()
import sys
path_=rp.get_path('optimization_pkg')+'/scripts'
sys.path.insert(0,path_)
from trajectory_pub import *
import time


class VizInHand:
    def __init__(self,obj_mesh='',finger_meshes=[''],init_node=True,t_name=''):
        self.obj_mesh=obj_mesh
        self.finger_meshes=finger_meshes
        # initialize node
        if(init_node):
            rospy.init_node('marker_viz')
        # initialize publisher
        self.obj_pub=rospy.Publisher('obj_marker',Marker,queue_size=1)
        if(t_name!=''):
            self.traj_server=trajectoryServer(100,robot_name='allegro',topic_name=t_name,init_node=False)
    def viz_js(self,js,t_name='regrasp_planner/in_grasp_traj'):
        js.name=['index_joint_0', 'index_joint_1', 'index_joint_2', 'index_joint_3', 'middle_joint_0', 'middle_joint_1', 'middle_joint_2', 'middle_joint_3', 'ring_joint_0', 'ring_joint_1', 'ring_joint_2', 'ring_joint_3', 'thumb_joint_0', 'thumb_joint_1', 'thumb_joint_2', 'thumb_joint_3']
        j_t=self.traj_server.get_jtraj(js)

        self.traj_server.viz_joint_traj(j_t,'allegro_mount')

    def viz_reachability(self,pose,meshes,arr=[0,1,2,3],t='R_fingers',a=0.1):
        f_pub=rospy.Publisher(t,MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        for i in arr:            
            obj_m=Marker()
            obj_m.header.frame_id='palm_link'
            obj_m.header.stamp=rospy.Time()
            obj_m.id=i
            obj_m.type=Marker.MESH_RESOURCE
            obj_m.mesh_resource=meshes[i]
            obj_m.action=Marker.ADD
            obj_m.scale.x=1.0
            obj_m.scale.y=1.0
            obj_m.scale.z=1.0
            obj_m.color.a=a
            obj_m.color.r=np.random.random_sample()
            obj_m.color.g=np.random.random_sample()
            obj_m.color.b=np.random.random_sample()
            
            obj_m.pose.position.x=pose[0]
            obj_m.pose.position.y=pose[1]
            obj_m.pose.position.z=pose[2]
            obj_m.pose.orientation.x=pose[3]
            obj_m.pose.orientation.y=pose[4]
            obj_m.pose.orientation.z=pose[5]
            obj_m.pose.orientation.w=pose[6]
            m_arr.markers.append(obj_m)
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<10):
            obj_m.header.stamp=rospy.Time()
            f_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1

        return False

    def viz_vertices(self,v,t='cs'):
        #print len(v)
        scale=0.001
        v_pub=rospy.Publisher(t,MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        r=1.0
        g=0.5
        b=0.0

        if(t=='cs'):
            r=0.0
            g=1.0
            b=0.0
        for i in range(len(v)):
            pose=np.ravel(v[i])
            obj_m=Marker()
            obj_m.header.frame_id='palm_link'
            obj_m.header.stamp=rospy.Time()
            obj_m.id=i
            obj_m.type=Marker.SPHERE
            obj_m.action=Marker.MODIFY
            obj_m.scale.x=scale
            obj_m.scale.y=scale
            obj_m.scale.z=scale
            obj_m.color.a=1.0
            obj_m.color.r=r
            obj_m.color.g=g
            obj_m.color.b=b
            obj_m.pose.position.x=pose[0]
            obj_m.pose.position.y=pose[1]
            obj_m.pose.position.z=pose[2]
            obj_m.pose.orientation.x=0.0
            obj_m.pose.orientation.y=0.0
            obj_m.pose.orientation.z=0.0
            obj_m.pose.orientation.w=1.0
            m_arr.markers.append(obj_m)
        
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<4):
            v_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1
                
        
    def viz_ellipse(self,f_1,f_2,a,v):
        thresh=a/200.0
        self.e_pub=rospy.Publisher('ellipse',MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        scale=0.002
        # Visualize foci:
        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=0
        obj_m.type=Marker.SPHERE
        obj_m.action=Marker.ADD
        obj_m.scale.x=scale
        obj_m.scale.y=scale
        obj_m.scale.z=scale
        obj_m.color.a=1.0
        obj_m.color.r=1.0
        obj_m.pose.position.x=f_1[0]
        obj_m.pose.position.y=f_1[1]
        obj_m.pose.position.z=f_1[2]
        obj_m.pose.orientation.x=0
        obj_m.pose.orientation.y=0
        obj_m.pose.orientation.z=0
        obj_m.pose.orientation.w=1.0
        m_arr.markers.append(obj_m)
        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=1
        obj_m.type=Marker.SPHERE
        obj_m.action=Marker.ADD
        obj_m.scale.x=scale
        obj_m.scale.y=scale
        obj_m.scale.z=scale
        obj_m.color.a=1.0
        obj_m.color.r=1.0
        obj_m.pose.position.x=f_2[0]
        obj_m.pose.position.y=f_2[1]
        obj_m.pose.position.z=f_2[2]
        obj_m.pose.orientation.x=0
        obj_m.pose.orientation.y=0
        obj_m.pose.orientation.z=0
        obj_m.pose.orientation.w=1.0

        m_arr.markers.append(obj_m)

        count=2
        # Sample points and visualize as ellipe:
        for j in range(len(v)):
            p=v[j]
            if(np.abs(np.sum((p-f_1)**2)+np.sum((p-f_2)**2)-2*a)<=thresh):
                obj_m=Marker()
                obj_m.header.frame_id='palm_link'
                obj_m.header.stamp=rospy.Time()
        
                obj_m.id=count
                obj_m.type=Marker.SPHERE
                obj_m.action=Marker.ADD
                obj_m.scale.x=scale
                obj_m.scale.y=scale
                obj_m.scale.z=scale
                obj_m.color.a=1.0
                obj_m.color.r=1.0
                obj_m.color.g=1.0
                obj_m.pose.position.x=p[0]
                obj_m.pose.position.y=p[1]
                obj_m.pose.position.z=p[2]
                obj_m.pose.orientation.x=0
                obj_m.pose.orientation.y=0
                obj_m.pose.orientation.z=0
                obj_m.pose.orientation.w=1.0
                m_arr.markers.append(obj_m)
                count+=1

        print count               
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<10):
            self.e_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1
                
    def viz_obj_sphere(self,pose):
        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=0
        obj_m.type=Marker.SPHERE
        obj_m.action=Marker.ADD
        obj_m.scale.x=0.2
        obj_m.scale.y=0.2
        obj_m.scale.z=0.2
        obj_m.color.a=1.0
        obj_m.pose.position.x=pose[0]
        obj_m.pose.position.y=pose[1]
        obj_m.pose.position.z=pose[2]
        obj_m.pose.orientation.x=pose[3]
        obj_m.pose.orientation.y=pose[4]
        obj_m.pose.orientation.z=pose[5]
        obj_m.pose.orientation.w=pose[6]
        while(not rospy.is_shutdown()):
            obj_m.header.stamp=rospy.Time()
            self.obj_pub.publish(obj_m)

    def viz_pts(self,cpts,t='cpt',rgb=[0.0,0.0,1.0],frame='palm_link'):
        scale=0.01
        self.v_pub=rospy.Publisher(t,MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        for i in range(len(cpts)):
            pt=cpts[i]
            obj_m=Marker()
            obj_m.header.frame_id=frame
            obj_m.header.stamp=rospy.Time()
            obj_m.id=i
            obj_m.type=Marker.SPHERE
            obj_m.action=Marker.ADD
            obj_m.scale.x=scale
            obj_m.scale.y=scale
            obj_m.scale.z=scale
            obj_m.color.a=1.0
            obj_m.color.r=rgb[0]
            obj_m.color.g=rgb[1]
            obj_m.color.b=rgb[2]
            obj_m.pose.position.x=pt[0]
            obj_m.pose.position.y=pt[1]
            obj_m.pose.position.z=pt[2]
            obj_m.pose.orientation.x=0.0
            obj_m.pose.orientation.y=0.0
            obj_m.pose.orientation.z=0.0
            obj_m.pose.orientation.w=1.0
            m_arr.markers.append(obj_m)
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<10):
            self.v_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1

    def viz_contact_normal(self, cpt,normal,rgb=[1.0,0.1,0.1],frame='palm_link',t='contact'):
        self.m_pub=rospy.Publisher(t,Marker,queue_size=1)
        arrow=Marker()

        arrow.header.frame_id=frame
        arrow.header.stamp=rospy.Time()
        arrow.id=0
        arrow.type=Marker.ARROW
        arrow.points.append(cpt)
        end_pt=np.ravel([cpt.x,cpt.y,cpt.z])+np.ravel([normal.x,normal.y,normal.z])*0.05
        end_p=Point()
        end_p.x=end_pt[0]
        end_p.y=end_pt[1]
        end_p.z=end_pt[2]
        
        arrow.points.append(end_p)
        arrow.color.a=1.0
        arrow.color.r=rgb[0]
        arrow.color.g=rgb[1]
        arrow.color.b=rgb[2]
        arrow.scale.x=0.005
        arrow.scale.y=0.01
        arrow.scale.z=0.005

        self.loop_rate=rospy.Rate(10)
        i=0
        while(not rospy.is_shutdown() and i<10):
            self.m_pub.publish(arrow)
            self.loop_rate.sleep()
            i+=1

    def viz_cpts(self,cpts,t='cpt',rgb=[1.0,0.1,0.1],frame='palm_link'):
        scale=0.01
        self.v_pub=rospy.Publisher(t,MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        for i in range(len(cpts)):
            pt=cpts[i]#.point
            obj_m=Marker()
            obj_m.header.frame_id=frame
            obj_m.header.stamp=rospy.Time()
            obj_m.id=i
            obj_m.type=Marker.SPHERE
            obj_m.action=Marker.ADD
            obj_m.scale.x=scale
            obj_m.scale.y=scale
            obj_m.scale.z=scale
            obj_m.color.a=1.0
            obj_m.color.r=rgb[0]
            obj_m.color.g=rgb[1]
            obj_m.color.b=rgb[2]
            obj_m.pose.position.x=pt.x
            obj_m.pose.position.y=pt.y
            obj_m.pose.position.z=pt.z
            obj_m.pose.orientation.x=0.0
            obj_m.pose.orientation.y=0.0
            obj_m.pose.orientation.z=0.0
            obj_m.pose.orientation.w=1.0
            m_arr.markers.append(obj_m)
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<10):
            self.v_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1
    

    def viz_nodes(self,pose_arr,initial_pose,desired_pose):
        scale=0.005
        self.v_pub=rospy.Publisher('rrt/tree',MarkerArray,queue_size=1)
        m_arr=MarkerArray()
        for i in range(len(pose_arr)):
            pose=pose_arr[i]
            obj_m=Marker()
            obj_m.header.frame_id='palm_link'
            obj_m.header.stamp=rospy.Time()
            obj_m.id=i
            obj_m.type=Marker.SPHERE
            obj_m.action=Marker.ADD
            obj_m.scale.x=scale
            obj_m.scale.y=scale
            obj_m.scale.z=scale
            obj_m.color.a=1.0
            obj_m.color.r=1.0
            obj_m.pose.position.x=pose[0]
            obj_m.pose.position.y=pose[1]
            obj_m.pose.position.z=pose[2]
            obj_m.pose.orientation.x=pose[3]
            obj_m.pose.orientation.y=pose[4]
            obj_m.pose.orientation.z=pose[5]
            obj_m.pose.orientation.w=pose[6]
            m_arr.markers.append(obj_m)
        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=len(pose_arr)+1
        obj_m.type=Marker.SPHERE
        obj_m.action=Marker.ADD
        obj_m.scale.x=scale
        obj_m.scale.y=scale
        obj_m.scale.z=scale
        obj_m.color.a=1.0
        obj_m.color.r=0.0
        obj_m.color.g=1.0
        obj_m.color.b=0.0
        obj_m.pose.position.x=initial_pose[0]
        obj_m.pose.position.y=initial_pose[1]
        obj_m.pose.position.z=initial_pose[2]
        obj_m.pose.orientation.x=initial_pose[3]
        obj_m.pose.orientation.y=initial_pose[4]
        obj_m.pose.orientation.z=initial_pose[5]
        obj_m.pose.orientation.w=initial_pose[6]
        m_arr.markers.append(obj_m)
        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=len(pose_arr)+2
        obj_m.type=Marker.SPHERE
        obj_m.action=Marker.ADD
        obj_m.scale.x=scale
        obj_m.scale.y=scale
        obj_m.scale.z=scale
        obj_m.color.a=1.0
        obj_m.color.r=0.0
        obj_m.color.g=0.0
        obj_m.color.b=1.0
        obj_m.pose.position.x=desired_pose[0]
        obj_m.pose.position.y=desired_pose[1]
        obj_m.pose.position.z=desired_pose[2]
        obj_m.pose.orientation.x=desired_pose[3]
        obj_m.pose.orientation.y=desired_pose[4]
        obj_m.pose.orientation.z=desired_pose[5]
        obj_m.pose.orientation.w=desired_pose[6]
        m_arr.markers.append(obj_m)

        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<10):
            self.v_pub.publish(m_arr)
            self.loop_rate.sleep()
            i+=1
            
    def viz_obj_mesh(self,pose,t='obj_marker',rgb=[0.5,0.5,0.5],a=0.4):
        obj_pub=rospy.Publisher(t,Marker,queue_size=1)

        obj_m=Marker()
        obj_m.header.frame_id='palm_link'
        obj_m.header.stamp=rospy.Time()
        obj_m.id=0
        obj_m.type=Marker.MESH_RESOURCE
        obj_m.mesh_resource=self.obj_mesh
        obj_m.action=Marker.ADD
        obj_m.scale.x=1.0
        obj_m.scale.y=1.0
        obj_m.scale.z=1.0
        obj_m.color.a=a
        obj_m.color.r=rgb[0]
        obj_m.color.g=rgb[1]
        obj_m.color.b=rgb[2]
        obj_m.pose=pose.pose
        i=0
        self.loop_rate=rospy.Rate(10)
        while(not rospy.is_shutdown() and i<5):
            obj_m.header.stamp=rospy.Time()
            obj_pub.publish(obj_m)
            self.loop_rate.sleep()
            i+=1

'''
if __name__=='__main__':
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file://'+grasp_loc+'/data/object_models/banana_bin.stl'
    f_meshes=['file://'+grasp_loc+'/data/robot_models/voxel_index.ply','file://'+grasp_loc+'/data/robot_models/voxel_middle.ply','file://'+grasp_loc+'/data/robot_models/voxel_ring.ply','file://'+grasp_loc+'/data/robot_models/voxel_thumb.ply']
    v_hand=VizInHand(o_mesh,f_meshes)
    obj_pose_=[0.05,0.0,0.0,0.1,0.1,-0.7,0.7]
    palm_pose_=[0.0,0.0,0.0,0.0,0.0,0.0,1.0]
    v_hand.viz_obj_mesh(obj_pose_)
    v_hand.viz_reachability(palm_pose_)
'''
