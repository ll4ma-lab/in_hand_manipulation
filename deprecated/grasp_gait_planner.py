import grasp_class
import viz_markers
import numpy as np

if __name__=='__main__':
    robot_meshes=['../data/robot_models/poisson_index.ply',
                  '../data/robot_models/poisson_middle.ply',
                  '../data/robot_models/poisson_ring.ply',
                  '../data/robot_models/poisson_thumb.ply']

    obj_mesh='../data/object_models/banana/meshes/poisson_mesh_simple.stl'
    # import object mesh, robot meshes
    grasp_g=grasp_class.grasp([0,1,2,3],robot_meshes)
    grasp_g.load_object(obj_mesh)

    # transform obj mesh
    rpy=[0.0,0.0*3.14/180.0,0.0*3.14/180.0]
    trans=[0.05,0.0,0.0]
    grasp_g.object.transform(rpy,trans)
    
    # transform finger mesh

    # Get contact regions
    grasp_g.generate_contact_surfaces(True)


    # compute geodesic distance between contact region and point on object mesh:
    d_idx=23
    #init_idx,_=grasp_g.get_new_sample(0,d_idx)
    init_idx=29
    cost=grasp_g.g_dist_compute(d_idx,init_idx,False)

    tree=[]

    # build RRT:
    f_idx=0
    g_pt=np.zeros(3)
    old_pt=grasp_g.object.mesh.vertices[init_idx]
    j=0
    
    while(j<10):
        if(grasp_g.contact_surfaces[f_idx]!=[]):
            t_cost=1000.0
            q_next=-1

            for i in range(40):
                v_idx,cost=grasp_g.get_new_sample(0,d_idx)
                #print cost
                if(cost<t_cost):
                    t_cost=cost
                    q_next=v_idx
                
            if(q_next not in tree):
                tree.append(q_next)
                # Move finger to new workspace:
                new_pt=grasp_g.object.mesh.vertices[tree[j]]
                delta_pt=old_pt-new_pt
                
                #g_pt=delta_pt
                # move finger
                grasp_g.finger_meshes[f_idx].transform(np.zeros(3),delta_pt)
                # Update workspace
                grasp_g.update_contact_surface(f_idx)
                
                old_pt=new_pt
                j+=1
                print t_cost
        else:
            j=11
    
    print tree
    
    # Visualize Object:
    grasp_loc='/home/bala/catkin_ws/src/grasp_gaits'
    o_mesh='file://'+grasp_loc+'/data/object_models/banana_bin.stl'
    f_meshes=['file://'+grasp_loc+'/data/robot_models/voxel_index.ply','file://'+grasp_loc+'/data/robot_models/voxel_middle.ply','file://'+grasp_loc+'/data/robot_models/voxel_ring.ply','file://'+grasp_loc+'/data/robot_models/voxel_thumb.ply']
    v_hand=viz_markers.VizInHand(o_mesh,f_meshes)
    o_pose=np.zeros(7)
    o_pose[-1]=1.0
    o_pose[0:3]=trans
    v_hand.viz_obj_mesh(o_pose)

    # Visualize RRT:

    # Show initial contact pt:
    initial_pose=np.zeros(7)
    initial_pose[6]=1.0
    initial_pose[0:3]=grasp_g.object.mesh.vertices[init_idx]
    desired_pose=np.zeros(7)
    desired_pose[6]=1.0
    desired_pose[0:3]=grasp_g.object.mesh.vertices[d_idx]
    # Show nodes:

    node_poses=[]
    for i in range(len(tree)):
        pose_=np.zeros(7)
        pose_[6]=1.0
        pose_[0:3]=grasp_g.object.mesh.vertices[tree[i]]
        node_poses.append(pose_)

    v_hand.viz_nodes(node_poses,initial_pose,desired_pose)
