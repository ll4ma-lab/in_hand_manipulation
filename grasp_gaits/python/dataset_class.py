# This file reads and writes to h5py
import numpy as np
import h5py
class datasetAppend:
    def __init__(self,file_name='test.hdf5'):
        f=h5py.File(file_name,'a')
        self.f=f
    def add_grasp(self,obj_name,grasp,g_no=-1):
        # grasp=[obj_pose,cpts,j_angles]
        #if(g_no==-1):
        g_name='g'+str(len(self.f[obj_name].keys()))
        #else:
        #    g_name='g'+str(g_no)
        subgrp=self.f[obj_name].create_group(g_name)
        subgrp.create_dataset('obj_pose',data=grasp[0])
        subgrp.create_dataset('cpts',data=grasp[1])
        subgrp.create_dataset('joint_angles',data=grasp[2])
    def read_grasp(self,obj_name,g_no):
        obj_pose=self.f[obj_name+'/g'+str(g_no)+'/obj_pose'][:]
        cpts=self.f[obj_name+'/g'+str(g_no)+'/cpts'][:]
        j0=self.f[obj_name+'/g'+str(g_no)+'/joint_angles'][:]
        return obj_pose,cpts,j0

class datasetRead:
    def __init__(self,file_name='test.hdf5'):
        f=h5py.File(file_name,'r')
        self.f=f
    def read_grasp(self,obj_name,g_no):
        obj_pose=self.f[obj_name+'/g'+str(g_no)+'/obj_pose'][:]
        cpts=self.f[obj_name+'/g'+str(g_no)+'/cpts'][:]
        j0=self.f[obj_name+'/g'+str(g_no)+'/joint_angles'][:]
        return obj_pose,cpts,j0
    def read_obj_file(self,obj_name):
        return self.f[obj_name].attrs['mesh_location'][:]
class datasetCreate:
    def __init__(self,file_name='test.hdf5'):
        f=h5py.File(file_name,'w')
        self.f=f
    def test(self):
        f=self.f
        grp=f.create_group('Banana')
        grp.attrs.create('mesh_location','banana.stl')
        grp.attrs.create('pose',[0.0,0.0,0.0,0.0,0.0,0.0,1.0])
        obj_poses=np.zeros(7)
        j_angles=np.zeros(16)
        cpts=np.zeros((4,3))
        subgrp=grp.create_group('g0')
        # Each dataset contains: obj_pose(1x7), cpts(4,3), joint_angles(1x16)
        #data=[obj_poses,cpts,j_angles]
        subgrp.create_dataset('obj_pose',data=obj_poses)
        subgrp.create_dataset('cpts',data=cpts)
        subgrp.create_dataset('joint_angles',data=j_angles)
       
'''
if __name__=='__main__':

    #d=datasetCreate()
    #d.test()
    d=datasetAppend()
    obj_poses=np.zeros(7)
    j_angles=np.zeros(16)
    cpts=np.zeros((4,3))
    g=[obj_poses,cpts,j_angles]
    #d.add_grasp('Banana',g)
    d.read_grasp('Banana',0)
'''
