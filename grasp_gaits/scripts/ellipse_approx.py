# This file approximates the contact surface to an ellipse
import numpy as np
import grasp_class
import viz_markers
import pyOpt
import tf


# formulate cost function:
def cost(q):
    a

# formulate constraint function:
def constraint(q):
    a
    
# formulate gradients:
# cost:

# constraint:

class snopt_wrapper:
    def __init__(self,data_arr):
        print 'Initialized snopt wrapper'
        self.data=data_arr
        
    def constraint(self,v,f_1,f_2,a):
        r=np.sum((f_1-v)**2)+np.sum((f_2-v)**2)-2*a
        return r
    
    def constraint_fn(self,q):
        f_1=q[0:3]
        f_2=q[3:6]
        a=q[-1]
        G=[]
        for i in range(len(self.data)):
            G.append(self.constraint(self.data[i],f_1,f_2,a))
        return G
    def cost_fn(self,q):
        r=abs(q[-1])#np.sum((q[0:3]-q[3:6]))**2
        return r
    
    def cost_fn_gradient(self,q):
        grad=np.zeros(7)
        #grad[0:3]=-2.0*(q[0:3]-q[3:6])
        #grad[3:6]=2.0*(q[0:3]-q[3:6])
        grad[-1]=1.0
        return list(grad)

    def constraint_grad(self,v,f_1,f_2,a):
        grad=np.zeros(7)
        grad[0:3]=2.0*(f_1-v)
        grad[3:6]=2.0*(f_2-v)
        grad[-1]=-2.0
        return grad
    def constraint_fn_gradient(self,q):
        G=[]
        f_1=q[0:3]
        f_2=q[3:6]
        a=q[-1]
        for i in range(len(self.data)):
            grad=np.zeros(7)
            grad+=self.constraint_grad(self.data[i],f_1,f_2,a)
            G.append(list(grad))
        return G
    
    def objfunc(self,q):
        f=self.cost_fn(q)
        g=list(self.constraint_fn(q))
        fail=0
        return f,g,fail

    def grad_obj_func(self,q,f,g):
        g_obj=np.array([self.cost_fn_gradient(q)])
        g_con=self.constraint_fn_gradient(q)
        fail=0
        return g_obj,g_con,fail


def fit_ellipse_pca(v):
    # Find origin and move all vertices to origin
    c=np.median(v,axis=0)
    v=v-c

    # Orient points
    ## Find major and minor axis using pca:
    cov_mat=np.matrix(v.T)*np.matrix(v)
    U,S,V = np.linalg.svd(cov_mat)


    U_r=U[:,0:2]
    print U_r.shape
    e_r=np.matrix(v)*np.matrix(U_r)

    # Find larges axis in all three dimensions:

    a=max(e_r[:,0])
    b=max(e_r[:,1])
    print a
    print b
    new_v=(np.matrix(U_r)*e_r.T).T+c
    return new_v
def fit_ellipse_opt(v,max_iter):
    wrap=snopt_wrapper(v)
    i=0
    f_1=np.array(v[1])
    f_2=np.array(v[-2])
    a=1e-1
    x=np.zeros(7)
    x[0:3]=f_1
    x[3:6]=f_2
    x[-1]=a

    # setup optimizaiton:
    # Bounds for variables:
    low_bounds=np.ones(7)*-0.5
    up_bounds=np.ones(7)*0.5

    cnst_bounds_up=np.ones(len(v))*0.0
    cnst_bounds_low=np.ones(len(v))*-10.0
    eq=np.zeros(len(v))
    q0=np.ravel(x)

    opt_prob=pyOpt.Optimization('TP37 Constrained Problem',wrap.objfunc)
    opt_prob.addObj('f')
    
    # Add variables
    opt_prob.addVarGroup('q',len(q0),type='c',value=q0,upper=up_bounds,lower=low_bounds)
    
    # In equality Constraints:
    opt_prob.addConGroup('g',len(v),type='i',lower=cnst_bounds_low,upper=cnst_bounds_up,equal=eq)
    
    method=getattr(pyOpt,'SNOPT')
    max_iter=1000
    #opt_options={'Iterations limit':max_iter,'Solution':'No','iPrint':0,'iSumm':0,'Problem Type':'Feasible point'}
    opt_options={'Iterations limit':max_iter,'Solution':'Yes','iPrint':1,'iSumm':1,'Problem Type':'Minimize'}
    optimizer=method(options=opt_options)
    
    #[fstr,xstr,inform]=optimizer(opt_prob,sens_type='FD')
    [fstr,xstr,inform]=optimizer(opt_prob,sens_type=wrap.grad_obj_func)
    print inform
    print 'Final cost: '+str(fstr)
    q_star=xstr
    return q_star[0:3],q_star[3:6],q_star[-1]
    
def median_cluster(v):
    mean=np.mean(v,axis=0)
    med=np.median(v,axis=0)
    new_v=[]
    for i in range(len(v)):
        if(np.sum(abs(v[i]-mean))>np.sum(abs(v[i]-med))):
           new_v.append(v[i])
    return new_v
            
    
    
if __name__=='__main__':
    # Get vertices of contact surface:
    robot_meshes=['../data/robot_models/poisson_index.ply',
                  '../data/robot_models/poisson_middle.ply',
                  '../data/robot_models/poisson_ring.ply',
                  '../data/robot_models/poisson_thumb.ply']


    obj_mesh='../data/object_models/box.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes)
    grasp_g.load_object(obj_mesh)

    # transform obj mesh
    rpy=[-90.0*3.14/180.0,0.0*3.14/180.0,-0.0*3.14/180.0]
    quat_=tf.transformations.quaternion_from_euler(rpy[0],rpy[1],rpy[2])
    
    quat=[quat_[-1],quat_[0],quat_[1],quat_[2]]
    print quat
    trans=[0.08,-0.08,0.015]
    grasp_g.object.transform(quat,trans)

    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file://'+grasp_loc+'/data/object_models/box_bin.stl'
    cs_meshes=['file://'+grasp_loc+'/scripts/f0.stl',
              'file://'+grasp_loc+'/scripts/f1.stl',
              'file://'+grasp_loc+'/scripts/f2.stl',
              'file://'+grasp_loc+'/scripts/f3.stl']
    
    f_meshes=['file://'+grasp_loc+'/data/robot_models/poisson_index.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_middle.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_ring.ply',
              'file://'+grasp_loc+'/data/robot_models/poisson_thumb.ply']

    v_m=viz_markers.VizInHand(o_mesh,cs_meshes)
    obj_pose=np.zeros(7)
    obj_pose[0:3]=trans

    obj_quat=np.zeros(4)
    obj_quat[0]=quat[1]
    obj_quat[1]=quat[2]
    obj_quat[2]=quat[3]
    obj_quat[-1]=quat[0]
    obj_pose[3:7]=obj_quat
    
    v_m.viz_obj_mesh(obj_pose)

    pose=np.zeros(7)
    pose[-1]=1.0

    v_m.viz_reachability(pose,f_meshes,[0,1,2,3],t='R_fingers',a=0.5)
    # transform finger mesh
    
    # Get contact regions
    grasp_g.generate_contact_surfaces(True)

    # Fit ellipse
    f=2
    vert=grasp_g.contact_surfaces[f].vertices
    # Filter vertices to find largest cluster:
    vert=median_cluster(vert)
    new_v=fit_ellipse_pca(vert)

    v_m.viz_reachability(pose,cs_meshes,[0,1,2,3],t='cs_fingers',a=0.8)
    #v_m.viz_ellipse(f_1,f_2,a,vert)
    v_m.viz_vertices(vert)
    v_m.viz_vertices(new_v,'new_v')
