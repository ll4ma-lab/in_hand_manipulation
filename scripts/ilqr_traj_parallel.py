 # This class has ILQR functions implemented from " Control limited Differential Programming- Yuval Tassa, Nicolas Mansard,Emo Todorov"

import numpy as np
from numpy.linalg import inv
# Parallel processing:
#from multiprocessing import Pool
import pathos
from pathos.pools import ThreadPool as TPool

class ILQR:
    def __init__(self,robot_model,cost,cost_gradient,final_cost,final_cost_gradient):
        self.robot=robot_model
        self.n=robot_model.n
        self.m=robot_model.m
        self.T=robot_model.T
        self.delta_t=robot_model.delta_t
        self.lamb_factor=np.sqrt(10.0)
        self.lamb_max=1e10
        self.max_iter=1000
        self.eps_converge=1e-5
        self.eps=1e-3
        self.cost=cost
        self.l_final=final_cost
        
        
    
    def final_cost_fd(self,x_des,x,u):
        # This final cost method uses finite differences to generate derivatives
        x=np.ravel(x)
        l=self.l_final(x_des,x,u,self.robot)
        l_x=np.zeros(self.n)
        l_xx=np.zeros((self.n,self.n))
        eps=self.eps
        for i in range(self.n):
            eps_array=np.zeros(self.n)
            eps_array[i]=eps
            l_x[i]=(self.l_final(x_des,x+eps_array,u,self.robot)-self.l_final(x_des,x-eps_array,u,self.robot))/(2.0*eps)
        for i in range(self.n):
            eps_array=np.zeros(self.n)
            eps_array[i]=eps
            for k in range(self.n):
                eps_arr2=np.zeros(self.n)
                eps_arr2[k]=eps
                d1=(self.l_final(x_des,x+eps_array+eps_arr2,u,self.robot)-self.l_final(x_des,x-eps_array+eps_arr2,u,self.robot))/(2.0*eps)
                d0=(self.l_final(x_des,x+eps_array-eps_arr2,u,self.robot)-self.l_final(x_des,x-eps_array-eps_arr2,u,self.robot))/(2.0*eps)
                l_xx[k,i]=(d1-d0)/(2.0*eps)

        return l,l_x,l_xx

    def cost_fd(self,x_des,x,u):
        # This function gets the derivatives of the immediate cost by finite differences.
        x=np.ravel(x)

        l=self.cost(x_des,x,u,self.robot)
        l_x=np.zeros(self.n)
        l_xx=np.zeros((self.n,self.n))
        l_u=np.zeros(self.m)
        eps=self.eps
        for i in range(self.n):
            eps_array=np.zeros(self.n)
            eps_array[i]=eps
            l_x[i]=(self.cost(x_des,x+eps_array,u,self.robot)-self.cost(x_des,x-eps_array,u,self.robot))/(2.0*eps)
        for i in range(self.n):
            eps_array=np.zeros(self.n)
            eps_array[i]=eps
            for k in range(self.n):
                eps_arr2=np.zeros(self.n)
                eps_arr2[k]=eps
                d1=(self.cost(x_des,x+eps_array+eps_arr2,u,self.robot)-self.cost(x_des,x-eps_array+eps_arr2,u,self.robot))/(2.0*eps)
                d0=(self.cost(x_des,x+eps_array-eps_arr2,u,self.robot)-self.cost(x_des,x-eps_array-eps_arr2,u,self.robot))/(2.0*eps)
                l_xx[k,i]=(d1-d0)/(2.0*eps)

        # Computing differences for change in input

        for i in range(self.m):
            eps_array=np.zeros(self.m)
            eps_array[i]=eps
            l_u[i]=(self.cost(x_des,x,u+eps_array,self.robot)-self.cost(x_des,x,u-eps_array,self.robot))/(2.0*eps)
        l_uu=np.zeros((self.m,self.m))
        l_ux=np.zeros((self.n,self.m))
        for i in range(self.m):
            eps_arr1=np.zeros((self.m))
            eps_arr1[i]=eps
            for j in range(self.n):
                eps_arr2=np.zeros(self.n)
                eps_arr2[j]=eps
                f1=(self.cost(x_des,x,u+eps_arr1,self.robot)-self.cost(x_des,x,u-eps_arr1,self.robot))/(2.0*eps)
                f0=(self.cost(x_des,x,u+eps_arr1,self.robot)-self.cost(x_des,x,u-eps_arr1,self.robot))/(2.0*eps)
                l_ux[j,i]=(f1-f0)/(2.0*eps)
            for k in range(self.m):
                eps_arr2=np.zeros(self.m)
                eps_arr2[k]=eps
                e1=(self.cost(x_des,x,u+eps_arr1+eps_arr2,self.robot)-self.cost(x_des,x,u-eps_arr1+eps_arr2,self.robot))/(2.0*eps)
                e0=(self.cost(x_des,x,u+eps_arr1-eps_arr2,self.robot)-self.cost(x_des,x,u-eps_arr1-eps_arr2,self.robot))/(2.0*eps)
                l_uu[k,i]=(e1-e0)/(2.0*eps)
            return l*self.delta_t,l_x*self.delta_t,l_xx*self.delta_t,l_u*self.delta_t,l_uu*self.delta_t,\
                   (l_ux*self.delta_t).T

    
    def finite_differences(self, x, u): 
        """ calculate gradient of plant dynamics using finite differences
        x np.array: the state of the system
        u np.array: the control signal 
        """  
        dof = u.shape[0]
        num_states = x.shape[0]

        A = np.zeros((num_states, num_states))
        B = np.zeros((num_states, dof))

        eps =self.eps # finite differences epsilon
       
        for ii in range(num_states):
            # calculate partial differential w.r.t. x
            eps_arr=np.zeros(num_states)
            eps_arr[ii]=eps
            A[:,ii]=(self.robot.sim.get_x_dot(x+eps_arr,u)-self.robot.sim.get_x_dot(x-eps_arr,u))/(2.0*eps) 

        for ii in range(dof):
            # calculate partial differential w.r.t. u
            eps_arr1=np.zeros(dof)
            eps_arr1[ii]=eps
            B[:,ii]=(self.robot.sim.get_x_dot(x,u+eps_arr1)-self.robot.sim.get_x_dot(x,u-eps_arr1))/(2.0*eps)
        return A, B

    def forward_simulation(self,data):
        x_des=data[0]
        x=data[1]
        u=data[2]
        l,l_x,l_xx,l_u,l_uu,l_ux=self.cost_fd(x_des,x,u)
        
        A,B=self.finite_differences(np.ravel(x.T),np.ravel(u))
        #A,B=self.robot.linearized_AB(np.ravel(x[self.robot.T-k-1,:]),np.ravel(u[self.robot.T-k-1]))
        f_x=np.eye(self.n,self.n)+A*self.delta_t
        f_u=B*self.delta_t

        return [l,l_x,l_xx,l_u,l_uu,l_ux,f_x,f_u,data[-1]]
    
    def controller(self,x0,x_des,u):
        N=self.T
        lamb=100.0
        sim_new_trajectory=True
        x=np.array([x0 for k in range(N)])#p.zeros((N,self.n))
        
        # Initializing variables
        l=np.zeros((N,1))
        l_x=np.zeros((N,self.n))
        l_xx=np.zeros((N,self.n,self.n))
        l_u=np.zeros((N,self.m))
        l_uu=np.zeros((N,self.m,self.m))
        l_ux=np.zeros((N,self.m,self.n))
        f_x=np.zeros((N,self.n,self.n))
        f_u=np.zeros((N,self.n,self.m))

        for ii in range(self.max_iter):
            # Forward rollout:
            if(sim_new_trajectory==True):
                cost=0.0
                for t in range(N-1):
                    cost+=self.cost(x_des[t],x[t,:],u[t],self.robot)*self.robot.delta_t
                cost+=self.l_final(x_des[-1],x[-1],u[-1],self.robot)

                # Calculate parameters:
                # Parallelizing:
                sc_pool=TPool(2)
                data_arr=[]
                for k in range(self.T):
                    data_arr.append([x_des[k],x[k],u[k],k])
                
                for res in sc_pool.map(self.forward_simulation,data_arr,chunksize=1):
                    k=res[-1]
                    l[k]=res[0]
                    l_x[k]=res[1]
                    l_xx[k]=res[2]
                    l_u[k]=res[3]
                    l_uu[k]=res[4]
                    l_ux[k]=res[5]
                    f_x[k]=res[6]
                    f_u[k]=res[7]
                    
                '''
                for k in range(self.T-1):
                '    l[k],l_x[k],l_xx[k],l_u[k],l_uu[k],l_ux[k],f_x[k],f_u[k]=self.forward_simulation(x_des[k],x[k],u[k])
                '''
                # Cost for final state:
                l[-1],l_x[-1],l_xx[-1]=self.final_cost_fd(x_des[-1],np.ravel(x[-1]),np.ravel(u[-1]))
                sim_new_trajectory=False
                 
            # Value function:
            

            V_x=np.matrix(l_x[-1]).T
            V_xx=np.matrix(l_xx[-1])
            k_ff=np.zeros((N,self.m))
            K=np.zeros((N,self.m,self.n))

            # Backward pass:
            for t in range(N-2,-1,-1):

                Q_uu=l_uu[t]+f_u[t].T*V_xx*f_u[t]
                
                # From ILQR code github:
                Q_uu_evals,Q_uu_evecs=np.linalg.eigh(Q_uu)
                Q_uu_evals[Q_uu_evals<0.0]=0.0
                Q_uu_evals=1.0/(Q_uu_evals+lamb)
                Q_uu_inv=Q_uu_evecs*np.diag(Q_uu_evals)*Q_uu_evecs.T
                Q_inv=Q_uu_inv*Q_uu*Q_uu_inv
              
                Q_ux=np.matrix(l_ux[t])+(f_u[t].T*V_xx*f_x[t])

                K[t]=Q_uu_inv*Q_ux
                V_xx=np.matrix(l_xx[t])+f_x[t].T*V_xx*f_x[t]-Q_ux.T*Q_inv*Q_ux
                Q_u=np.matrix(l_u[t]).T+f_u[t].T*V_x

                k_ff[t]=np.ravel(Q_uu_inv*Q_u)
                
        
        
        
                V_x=np.matrix(l_x[t]).T+f_x[t].T*V_x-(Q_ux.T*Q_inv*Q_u)
              

            # Calculating new input:
            u_new=np.zeros((N,self.m))
            x_new_arr=np.zeros((N,self.n))
            x_new_arr[0]=np.ravel(x[0,:])
            cost_new=0.0
            for t in range(N-1):
                u_new[t]=u[t]-k_ff[t]-np.dot(K[t],x_new_arr[t]-np.ravel(x[t,:]))
                cost_new+=self.cost(x_des[t],x_new_arr[t,:],u_new[t],self.robot)*self.robot.delta_t
                x_new_arr[t+1]=x_new_arr[t]+self.robot.sim.get_x_dot(x_new_arr[t],u_new[t])*self.delta_t
            cost_new+=self.l_final(x_des[-1],x_new_arr[-1],u[-1],self.robot)

            print "new_cost "+str(cost_new)
            if(cost_new<cost):
                lamb/=self.lamb_factor
                x=x_new_arr
                u=u_new
                old_cost=cost
                cost=cost_new
                sim_new_trajectory=True
                if(ii>0 and (abs(cost-old_cost)/cost<self.eps_converge)):
                    print 'Converged'
                    break
            else:
                lamb*=self.lamb_factor
                if(lamb>self.lamb_max):
                    print " No convergence "+str(np.log(lamb))+" iter: "+str(ii)
                    break
        # Writing torques to file:
        '''
        f=open("torque_ilqr","w")
        for i in range(self.T):
            for k in range(self.m):
                f.write(str(u_new[i,k]))
                f.write(' ')
            f.write('\n')
        '''
        #return u_new,x_new_arr
        return k_ff,K,x_new_arr,u_new
