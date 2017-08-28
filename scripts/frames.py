import numpy as np
import PyKDL

class transformMat:
    def __init__(self):
        #initialized mat
        self.i=0
    def T_pose(self,pose):
        T=np.eye(4)
        R=PyKDL.Rotation()
        if(len(pose)==7):
            R=R.Quaternion(pose[3],pose[4],pose[5],pose[6])
        else:
            R=R.RPY(pose[3],pose[4],pose[5])
        T[0:3,3]=pose[0:3]
        for k in range(3):
            for l in range(3):
                T[k,l]=R[k,l]
        return T

    def pose_T(self,T,RPY=True):
        if(RPY):
            pose=np.zeros(6)
        else:
            pose=np.zeros(7)
        pose[0:3]=T[0:3,3]

        R=PyKDL.Rotation(T[0,0],T[0,1],T[0,2],
                         T[1,0],T[1,1],T[1,2],
                         T[2,0],T[2,1],T[2,2])
        if(RPY==False):
            pose[3:7]=R.GetQuaternion()
        else:
            pose[3:6]=R.GetRPY()
        
        return pose
    
    def quat_RPY(self,rpy):
        R=PyKDL.Rotation()
        R=R.RPY(rpy[0],rpy[1],rpy[2])
        quat=np.zeros(4)
        quat=R.GetQuaternion()
        return quat

    def rigid_svd(self,pts_a,pts_b):
        # Find a rigid transformation from  pts_a to pts_b
        N=len(pts_a)
        # Convert points to a matrix:
        # pts=3 x N
        pts_a=np.matrix(pts_a).T
        pts_b=np.matrix(pts_b).T
        
        centroid_a=np.mean(pts_a,axis=1)
        centroid_b=np.mean(pts_b,axis=1)
        # centroids: 3x1
        
    
        # move points to their origin:
        #pts_a-=centroid_a
        #pts_b-=centroid_b

        H=np.zeros((3,3))
        # make points as matricesL
        for i in range(N):
            H+=(pts_a[:,i]-centroid_a)*(pts_b[:,i]-centroid_b).T
        # Do svd:
        U,S,V=np.linalg.svd(H)
        # Rotation matrix:
        R=np.matrix(V).T*np.matrix(U).T

        # Check for reflection case:
        if(np.linalg.det(R)<0):
            R[0:3,2]=R[0:3,2]*-1.0
        # translation:
        t=np.ravel(-R*centroid_a)
        t+=np.ravel(centroid_b)
    
        # T matrix:
        T=np.eye(4)
        T[0:3,0:3]=R
        T[0:3,3]=np.ravel(t)


        # Transform points and check error:
        
        # pose:
        pose=self.pose_T(T)
        return pose
