import pickle
import rospy
import numpy as np

if __name__=='__main__':
    f_name_="friction_exp_final.p"
    # check for pickle file, if missing create a new file
    #obj_name=raw_input("Type in object name")
    #if os.path.isfile(f_name_):
    pic_data = pickle.load( open( f_name_, "rb" ) )
    keys = pic_data.keys()
    keys.sort()
    for j in keys:
        data = pic_data[j]
        mu = []
        for i in range(len(data)):
            mu.append(data[i]['mu'])
        print np.median(mu)
    #print np.rad2deg(np.arctan(np.median(mu)))
    #print np.median(mu)
    #print np.tan(np.deg2rad(24))
