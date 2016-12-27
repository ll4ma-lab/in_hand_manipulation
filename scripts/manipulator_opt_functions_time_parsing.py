#!/usr/bin/env python
import numpy as np
#import pathos
#from pathos.pools import ThreadPool as TPool
#import psutil

class OptFns:
    def __init__(self,immediate_cost,final_cost,grad,grad_final,robot_model,x_d,x0):
        self.eps=1e-15
        self.delta_t=robot_model.delta_t
        self.cost=immediate_cost
        self.final_cost=final_cost
        self.gradient=grad
        self.gradient_final=grad_final
        self.robot=robot_model
        self.x_d=x_d
        self.u0=x0
        self.n=len(x_d[0])
        self.m=robot_model.m
       
    def multi_cost(self,data):
        return self.cost(data[0:self.n],data[self.n:self.n+self.m],self.robot,data[self.n+self.m:])
    
    def objectiveFunction(self,q):
        '''
        q - decision variable vector of the form (U[0], X[1], U[1],..., U[H-1], X[H])
        H - time horizon
        n - system state dimension
        m - control dimension
        x0 - inital state
        x_d - desired trajectory
        dyn_model - model simulating system dynamics of the form x[k+1] = dyn_model.predict(x[k], u[k])
        '''
    
        x,u=self.get_x_u_from_q(q)
        #m=self.robot.m
        cost=0.0
        x_d=self.x_d
        u_arr=u[1:-1]
        x_d_arr=x_d[:-1]
        t=np.array([range(self.robot.T-1)]).T

        #print u_arr[0]
        data_arr=np.concatenate((x_d_arr,u_arr),axis=1)
        data_arr=list(np.concatenate((data_arr,t),axis=1))

        #cost_arr=map(self.multi_cost,data_arr)
        cost_arr=[self.multi_cost(k) for k in data_arr]
        cost=sum(cost_arr)

        cost+=self.final_cost(x_d[-1],u[-1],self.robot)

        #print cost
        
        return cost


    def multi_gradient(self,data):
        return self.gradient(data[0:self.n],data[self.n:self.n+self.m],self.robot,data[self.n+self.m:])
        
    def objectiveGradient(self,q):
        ## TODO: Make this generic for any robot model 
        #  print 'Computing Objective Gradient'
        
        #m=self.robot.m
        x, u = self.get_x_u_from_q(q)
        
        
        #k=len(q)-2*step
        x_d=self.x_d
        
        
        u_arr=u[1:-1]
        x_d_arr=x_d[:-1]
        t=np.array([range(self.robot.T-1)]).T

        data_arr=np.concatenate((x_d_arr,u_arr),axis=1)
        data_arr=list(np.concatenate((data_arr,t),axis=1))

        #gradient_arr=map(self.multi_gradient,data_arr)
        gradient_arr=[self.multi_gradient(k) for k in data_arr]
        gradient=np.ravel(gradient_arr)

        gradient=np.append(gradient,self.gradient_final(x_d[-1],u[-1],self.robot))
        return gradient

    def velocity_constraint(self,q):
        x,u= self.get_x_u_from_q(q)
        G=np.zeros(len(q))
        for k in xrange(self.robot.T):
            G[k*self.robot.m:(k+1)*self.robot.m]=np.ravel(u[k+1]-u[k])#/self.delta_t
        return G

    def velocity_gradient(self,q):
        x,u= self.get_x_u_from_q(q)
        J=np.zeros(((self.robot.T)*(self.robot.m),q.size))
        m=self.robot.m
        velocity_grad=np.zeros((m,m*2))

        for k in range(m):        
            velocity_grad[k,k]=-1.0#/(self.delta_t)
            velocity_grad[k,k+m]=1.0#/(self.delta_t)

        J[0:m,0:m]=np.eye(m)*(1.0)
        for k in range(1,self.robot.T):
            row_start = (k)*m
            row_stop = row_start+m

            col_start =(m)*(k-1)
            col_stop = col_start+2*m
            # Each Jacobian will have gradients of the dynamics model
            J[row_start:row_stop,col_start:col_stop]+=velocity_grad
           
        return J    

    def get_x_u_from_q(self,q):
        m=self.robot.m
        u=np.array([[0.0 for k in range(m)] for i in range(self.robot.T+1)])
        step=m
        u[0,:]=self.u0
        for k in range(self.robot.T):
            u_start=k*step
            u_stop=u_start+m
            u[k+1,:]=q[u_start:u_stop]
        u=np.array(u)
        x=[]
        return x,u        

    def get_q(self,x,u):
        q=u[1:].ravel()
        return q


    def rolling_constraint(self,q):
        x,u= self.get_x_u_from_q(q)
        G=np.zeros(self.robot.T)
        for k in xrange(self.robot.T):
            diff=np.ravel(self.robot.end_effector_pose(u[k+1,0:4],0)[0:3,3]-self.robot.end_effector_pose(u[k+1,4:8],3)[0:3,3])
            G[k]=(np.dot(diff,diff))
        return G
    def rolling_gradient(self,q):
        J=np.zeros(((self.robot.T),q.size))
        x,u= self.get_x_u_from_q(q)
        m=self.robot.m
        for i in range(self.robot.T):
            diff=np.ravel(self.robot.end_effector_pose(u[i+1,0:4],0)[0:3,3]-self.robot.end_effector_pose(u[i+1,4:8],3)[0:3,3])
            J[i,m*i:m*i+m/2]=2.0*(diff)*self.robot.jacobian(u[i+1,0:4],0)
            J[i,m*i+m/2:m*i+m]=2.0*(diff)*self.robot.jacobian(u[i+1,4:8],3)
        return J
