# This file converts a posestamped message to a tf transform
import rospy
import tf
from dbot_ros_msgs.msg import ObjectState

def send_transform(pose,frame):
    br = tf.TransformBroadcaster()
    br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                    (pose.orientation.x,pose.orientation.y, pose.orientation.z,pose.orientation.w),
                     rospy.Time.now(),
                     'dbot/obj',
                     frame)


def pose_cb(data):
    pose=data.pose.pose
    send_transform(pose,data.pose.header.frame_id)
    
if __name__=='__main__':
    rospy.init_node('pose_tf_node')
    rospy.Subscriber('/particle_tracker/object_state',ObjectState,pose_cb)
    r=rospy.Rate(50)
    while(not rospy.is_shutdown()):
        r.sleep()
