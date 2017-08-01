#!/usr/bin/env python
import numpy as np
_DISPLAY_RATE = 50
from object_list import *
import sys
class IlqrTester:
    ''' This class takes in joint position commands, delta_t and publishes the data to the robot'''
    

    def read_file(self,traj_no):
        
        self.names_index=np.array([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15])
        
        #file_name="objects.csv"
        #folder_to_write="dist_20/ilqr/"
        #obj=ObjectList(file_name,folder_to_write)
        #f_name=obj.arr[traj_no].f_name
        #folder=f_name+"/"
        u_file=open('ilqr_u','r')
        self.u_array=np.load(u_file)
        u_file.close()
        x_file=open('ilqr_x','r')
        self.x_array=np.load(x_file)
        x_file.close()

        kff_file=open('kff_gains','r')
        K_file=open('K_gains','r')
        self.k_ff=np.load(kff_file)
        self.K=np.load(K_file)
        self.delta_t=60.0
    def run_allegro_pub(self):
       
        
        for k in xrange(len(self.x_array[:,0])-1):          
            #print self.curr_pos
            print np.trace(self.K)
            

def test_allegro_traj(listen=True, publish_prefix='', listen_prefix=''):
    tj=int(sys.argv[1])
    jt=IlqrTester()
    jt.read_file(tj)
    jt.run_allegro_pub()

if __name__=='__main__':

    test_allegro_traj()
        
