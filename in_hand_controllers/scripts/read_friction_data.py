import pickle
import rospy


if __name__=='__main__':
    f_name_="friction_exp_final.p"
    # check for pickle file, if missing create a new file
    #obj_name=raw_input("Type in object name")
    #if os.path.isfile(f_name_):
    pic_data = pickle.load( open( f_name_, "rb" ) )
    print pic_data.keys()
    print pic_data['ss_plastic_1']
