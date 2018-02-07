import pickle
import numpy as np
from trajectory_msgs.msg import JointTrajectory
'''
class pickleData:
    def read_file(self,file_name):
        file_ = open(file_name,'rb')
        obj=pickle.load(file_)
        return obj
    
    def get_config(self,obj,i):
        obj_pose=obj[i][0]
        js=obj[i][1]
        cpt=obj[i][3]
        des_contact_pt=obj[i][2]
        return obj_pose,js,cpt,des_contact_pt

'''
class gaitObj:
    def __init__(self,i_js,f_js,init_o_pose,final_o_pose,cpt,des_cpt,r_cpt,fg=False,ig=False,f=0,traj=JointTrajectory(),mesh_f="h",mesh_stl="j"):
        self.mesh_f=mesh_f
        self.mesh_stl=mesh_stl
        self.i_js=i_js
        self.f_js=f_js
        self.init_o_pose=init_o_pose
        self.final_o_pose=final_o_pose
        self.fg=fg
        self.ig=ig
        self.f=f
        self.des_cpt=des_cpt
        self.cpt=cpt
        self.r_cpt=r_cpt
        self.traj=traj

class gaitFile:
    def read_file(self,obj_file):
        g=pickle.load(open(obj_file,"rb"))
        return g

'''
if __name__=='__main__':
    obj_name='mustard_7.p'
    obj_file='../python/r_mustard_7.p'
    mesh_f='/home/bala/object_models/mustard_bottle/google_16k/vertices.ply'
    mesh_stl='file:///home/bala/object_models/mustard_bottle/google_16k/nontextured.stl'

    
    p_data=pickleData()

    obj_dat=p_data.read_file(obj_file)
    g=[]
    for i in range(len(obj_dat)-2):
        init_o_pose,i_js,cpt,des_cpt=p_data.get_config(obj_dat,i)
        final_o_pose,final_js,r_cpt,des_cpt=p_data.get_config(obj_dat,i+1)
        # Check if its fg or ig:
        i_pose=np.array([init_o_pose.pose.position.x, init_o_pose.pose.position.y,init_o_pose.pose.position.z])
        o_pose=np.array([final_o_pose.pose.position.x, final_o_pose.pose.position.y,final_o_pose.pose.position.z])
        fg=False
        ig=False
        f=-1
        if(np.linalg.norm(i_pose-o_pose)>0.00001):
            fg=False
            ig=True
        else:
            for k in range(4):
                i_cpt=np.array(i_js.position)[k*4:k*4+4]
                f_cpt=np.array(final_js.position)[k*4:k*4+4]
                if(np.linalg.norm(i_cpt-f_cpt)>0.0000001):
                    f=k
                    fg=True
                    break
        g1=gaitObj(i_js,final_js,init_o_pose,final_o_pose,cpt,des_cpt,r_cpt,mesh_f=mesh_f,mesh_stl=mesh_stl, fg=fg,ig=ig,f=f)
        g.append(g1)

    pickle.dump(g,open(obj_name,"wb"))
'''
