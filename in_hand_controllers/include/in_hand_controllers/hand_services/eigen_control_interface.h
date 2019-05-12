/*
 * Copyright (C) 2018  Balakumar Sundaralingam, LL4MA Lab, University of Utah
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <hand_services/control_interface/control_interface.h>
#include <grasp_utils/eigen_grasp.hpp>
namespace hand_services
{
namespace control_interface
{
/// Class for interfacing with a robot hand assuming a low level controller runs PD for position and direct torque control
class eigenControlInterface: public controlInterface
{
public:

  // declare functions from inherited base class:
  /** 
   * Class initializer
   * 
   * @param ns namespace for the nodehandle
   * @param robot_interface hand_interface pointer
   * @param loop_rate control loop rate
   * 
   * @return None (class initializer)
   */
  eigenControlInterface(string ns, robot_interface::RobotInterface *robot_interface,double loop_rate): controlInterface(ns,robot_interface,loop_rate){ }

  
  bool configureHook();

  //using controlInterface::externalCmdCallback;
protected:

  /** 
   * hand command request callback function
   * 
   * @param req service request data
   * @param resp service response data
   */
  bool handCommandEigen(hand_services::SendHandCommand::Request &req, hand_services::SendHandCommand::Response &resp);

  /** 
   * Moves the joints in eigen space until contact is reached
   * 
   * @param tactile_sensing 
   * @param fingers 
   * @param eigen_amplitudes 
   * 
   * @return 
   */
  bool approachContactEigen(const  bool &tactile_sensing, const vector<int> &fingers, const vector<double> &eigen_amplitudes,Eigen::VectorXd &last_cmd);

  bool increaseStiffnessEigen(const vector<int> &fingers,const Eigen::VectorXd &q_des,const vector<double> &eigen_amplitudes, Eigen::VectorXd &last_jcmd);
  bool eigen_space_;
  grasping::eigenGrasp eigen_grasp_;

  void externalCmdCallback1(sensor_msgs::JointState msg);
};
}
}
