# this node receives a joint state for the robot through a service call and publishes for viewing in RVIZ. This allows for initializing robot joint state for planning
import rospy
from in_hand_manipulation_msgs.srv import UpdateRobotConfig
from sensor_msgs.msg import JointState

class simState(object):
    def __init__(self):
        #
        rospy.init_node('joint_sim')
        #create publisher
        self.pub = rospy.Publisher('/lbr4_allegro/joint_states',JointState, queue_size=10)
        self.srv = rospy.Service('/in_hand_utils/update_robot_state', UpdateRobotConfig, self.srv_call)

        #create service call
        self.joint_state = None
        self.loop_rate = rospy.Rate(100)
    def run_loop(self):
        while(not rospy.is_shutdown()):
            if(self.joint_state is not None):
                self.joint_state.header.stamp = rospy.Time.now()
                self.pub.publish(self.joint_state)
            self.loop_rate.sleep()
    def srv_call(self,req):
        self.joint_state = req.robot_state
        #res.success = True
        return True
if __name__=='__main__':
    # create ros node
    sim_node = simState()

    sim_node.run_loop()
    # create service
