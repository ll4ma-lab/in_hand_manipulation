    def hybrid_impedance_ctrl(self):
        des_pdc=70.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                u_cmd=self.u.copy()
                self.ind_des_pose=(self.hand.robot_kdl.end_effector_pose_array(self.u[0:4],0)[0:3]           +np.array([0.0,0.01,0.0]))
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                self.initial_offsets=[np.zeros(3) for i in range(4)]
                for i in self.fingers:
                    self.initial_offsets[i]=(self.initial_vf
                                             -self.hand.robot_kdl.end_effector_pose_array(self.u[i*4:i*4+4],i)[0:3])

            else:
                u_cmd=self.u_cmd.copy()
            
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            u=np.array(self.hand.joint_state.position)
            u_dot=np.array(self.hand.joint_state.velocity)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                finger_pose=self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3]
                err=(vf-finger_pose)*np.array([1.0,0.0,1.0])
                #err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)

                K_p=30*self.f_gain[i]*float(des_pdc-pdc_arr[i])
                if(K_p<=0.0):
                    K_p=0.0
                K_d=0.1
                # Maintain contact position
                pos_err=self.initial_offsets[i]-(vf-finger_pose)
                #if(i==0):
                    #print pos_err
                K_pos=0.0
                theta_dot=(np.ravel(K_p*J.transpose()*np.matrix(err).transpose())-K_d*u_dot[i*4:i*4+4]
                           -np.ravel(K_pos*J.transpose()*np.matrix(pos_err).transpose()))
                # Moving object to desired position:
                err=(self.initial_vf-vf)
                K_vf=200.0
                theta_dot+=np.ravel(K_vf*J.transpose()*np.matrix(err).transpose())
                #print err

                # Maintain initial contact location?:
                
                # Attitude controller:

                
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0
                
            '''
            if(0 not in self.fingers):
                i=0
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                pos_err=(self.ind_des_pose-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([0.0,1.0,0.0])
                pos_err=pos_err
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)
                
                K_p=self.f_gain[i]*float(10.0-pdc_arr[i])
                K_slide=100.0

                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                psi_dot=np.ravel(K_slide*J.transpose()*np.matrix(pos_err).transpose())

                u_cmd[i*4:i*4+4]+=(theta_dot+psi_dot)/100.0
            '''
            self.hand.publish_joints_ctrl(u_cmd)

            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True

    def contact_ctrl(self):
        des_pdc=50.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                u_cmd=self.u.copy()
                self.ind_des_pose=(self.hand.robot_kdl.end_effector_pose_array(self.u[0:4],0)[0:3]           +np.array([0.0,0.01,0.0]))
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                self.initial_offsets=[np.zeros(3) for i in range(4)]
                for i in self.fingers:
                    self.initial_offsets[i]=(self.initial_vf
                                             -self.hand.robot_kdl.end_effector_pose_array(self.u[i*4:i*4+4],i)[0:3])

            else:
                u_cmd=self.u_cmd.copy()
            
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            u=np.array(self.hand.joint_state.position)
            u_dot=np.array(self.hand.joint_state.velocity)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            contact_normals=self.biotac.contact_normal
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                #finger_pose=self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3]
                #err=(vf-finger_pose)*np.array([1.0,1.0,1.0])
                
                if(contact_normals[i][0]!=0.0):
                    err=contact_normals[i]/np.linalg.norm(contact_normals[i])
                else:
                    err=contact_normals[i]

                

                K_p=0.1*self.f_gain[i]*float(des_pdc-pdc_arr[i])
                K_d=1.0
                # Maintain contact position
                #if(i==0):
                    #print np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                    
                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())-K_d*u_dot[i*4:i*4+4]
                       
                # Moving object to desired position:
                err=(self.initial_vf-vf)
                K_vf=00.0
                #theta_dot+=np.ravel(K_vf*J.transpose()*np.matrix(err).transpose())
                #print err

                # Maintain initial contact location?:
                
                # Attitude controller:

                
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0

            self.hand.publish_joints_ctrl(u_cmd)

            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True

    
    def impedance_ctrl(self):
        des_pdc=50.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                u_cmd=self.u.copy()
            else:
                u_cmd=self.u_cmd

                
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            u=np.array(self.hand.joint_state.position)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])
                K_p=self.f_gain[i]*float(des_pdc-pdc_arr[i])
                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                err=self.initial_vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3]
                psi_dot=np.ravel(self.vf_gain*K_p*J.transpose()*np.matrix(err).transpose())
                u_cmd[i*4:i*4+4]+=(theta_dot+psi_dot)/100.0
                u_cmd[i*4]=self.u[i*4]
                if(i==0):
                    print 'theta:'
                    print theta_dot
                    print psi_dot


            self.hand.publish_joints_ctrl(u_cmd)
            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True

    def impedance_norm_ctrl(self):
        des_pdc=50.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                u_cmd=self.u.copy()
            else:
                u_cmd=self.u_cmd

                
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            u=np.array(self.hand.joint_state.position)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                err=np.linalg.norm(err)*err/np.linalg.norm(err)
                K_p=self.f_gain[i]*float(des_pdc-pdc_arr[i])
                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0
                

            self.hand.publish_joints_ctrl(u_cmd)
            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True

    def impedance_rest_length_ctrl(self):
        des_pdc=60.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                u_cmd=self.u.copy()
                self.ind_des_pose=(self.hand.robot_kdl.end_effector_pose_array(self.u[0:4],0)[0:3]           +np.array([0.0,0.01,0.0]))
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                
            else:
                u_cmd=self.u_cmd.copy()
            
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()
            u=np.array(self.hand.joint_state.position)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                err=(self.initial_vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)

                K_p=self.f_gain[i]*float(des_pdc-pdc_arr[i])
                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                theta_dot[0]=0.0
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0

            if(0 not in self.fingers):
                i=0
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                pos_err=(self.ind_des_pose-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([0.0,1.0,0.0])
                pos_err=pos_err
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)
                
                K_p=self.f_gain[i]*float(10.0-pdc_arr[i])
                K_slide=100.0

                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                psi_dot=np.ravel(K_slide*J.transpose()*np.matrix(pos_err).transpose())

                u_cmd[i*4:i*4+4]+=(theta_dot+psi_dot)/100.0
            
            self.hand.publish_joints_ctrl(u_cmd)

            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True
        
    def tahara_impedance_ctrl(self):
        des_pdc=150.0
        if(self.start_impedance==True):
            if(self.got_init_joint_angles==False):
                self.u=np.array(self.hand.joint_state.position)
                u_cmd=self.u.copy()
                self.ind_des_pose=(self.hand.robot_kdl.end_effector_pose_array(self.u[0:4],0)[0:3]           +np.array([0.0,0.01,0.0]))
                self.initial_vf=self.hand.robot_kdl.min_object_pose(self.u,self.fingers)[0:3]
                self.got_init_joint_angles=True
                
            else:
                u_cmd=self.u_cmd.copy()
            
            # Get biotac pressure reading:
            pdc_arr=self.biotac.get_pdc()

            u=np.array(self.hand.joint_state.position)
            u_dot=np.array(self.hand.joint_state.velocity)
            # virtual frame:
            vf=self.hand.robot_kdl.min_object_pose(u,self.fingers)[0:3]
            # update control law and send to robot
            for i in self.fingers:
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                #err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)

                K_p=self.f_gain[i]*float(des_pdc-pdc_arr[i])
                #if(K_p<0.0):
                #    K_p=0.0
                if(i==2):
                    print err
                K_d=0.1
                # Adding desired force from Tahara model:
                #?
                
                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())-K_d*u_dot[i*4:i*4+4]
                # Moving object to desired position:
                err=(self.initial_vf-vf)
                K_vf=150.0
                theta_dot+=np.ravel(K_vf*J.transpose()*np.matrix(err).transpose())
                #print err

                # Maintain initial contact location?:
                
                # Attitude controller:

                
                u_cmd[i*4:i*4+4]+=(theta_dot)/100.0

            '''
            if(0 not in self.fingers):
                i=0
                J=self.hand.robot_kdl.jacobian(u[i*4:i*4+4],i)
                pos_err=(self.ind_des_pose-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([0.0,1.0,0.0])
                pos_err=pos_err
                err=(vf-self.hand.robot_kdl.end_effector_pose_array(u[i*4:i*4+4],i)[0:3])*np.array([1.0,0.0,1.0])
                err=(np.linalg.norm(err)-self.dist[i])*err/np.linalg.norm(err)
                
                K_p=self.f_gain[i]*float(10.0-pdc_arr[i])
                K_slide=100.0

                theta_dot=np.ravel(K_p*J.transpose()*np.matrix(err).transpose())
                psi_dot=np.ravel(K_slide*J.transpose()*np.matrix(pos_err).transpose())

                u_cmd[i*4:i*4+4]+=(theta_dot+psi_dot)/100.0
            '''
            self.hand.publish_joints_ctrl(u_cmd)

            self.u_cmd=u_cmd.copy()
        else:
            # TODO: Switch to grav comp
            return True
