# Computes the current grasp error and prints to screen
import tf
import rospy
import numpy as np
DISP_RATE_=2

def main():
    rospy.init_node('grasp_err_node')
    rate=rospy.Rate(10.0)
    listener=tf.TransformListener()
    th_data_=False
    obj_data_=False
    i=0
    while not rospy.is_shutdown():
        i+=1
        try:
            (th_pose,_)=listener.lookupTransform('rigid','i',rospy.Time(0))
            th_data_=True
        except:
            continue
        try:
            (obj_pose,_)=listener.lookupTransform('obj_ar','i',rospy.Time(0))
            obj_data_=True
        except:
            continue
            
        if(th_data_==True and obj_data_==True):
            if(i%DISP_RATE_==1):
                print 'Thumb_err: '+"%0.4f" % (np.linalg.norm(th_pose))+' Obj_err: '+"%0.4f" % (np.linalg.norm(obj_pose))
    
        rate.sleep()
    
if __name__=='__main__':
    main()
