import rosbag
from tf_bag import BagTfTransformer


bag_folder='/media/bala/data_giskard/in_grasp_journal/feedback'+'/'
bag_file=bag_folder+'Apple_0_0.bag'
#bag_name= bag_folder+obj_.name+'_'+str(idx)+'_'+str(t_no)+'.bag'


if __name__=='__main__':
    bag = rosbag.Bag(bag_file)
    bag_transformer = BagTfTransformer(bag)
    e_time=bag_transformer.getEndTime()
    print e_time
    # marker to object
    translation, quaternion = bag_transformer.lookupTransform('dbot/obj', '/dbot/obj_pose',e_time)

    # camera to marker:
    translation, quaternion = bag_transformer.lookupTransform('/camera_link','dbot/obj',e_time)

