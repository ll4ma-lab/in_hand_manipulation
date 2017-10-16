import numpy as np
import pickle

def clean_data(file_name):
    file_ = open(file_name,'rb')
    object_file = pickle.load(file_)
    file_.close()
    G_arr=object_file
    d=[]
    for i in range(1,len(G_arr)-1):
        G=G_arr[i]
        c_pts=G[3]
        d_pts=G[2]
        diff=np.zeros(4)
        for i in range(len(d_pts)):
            diff[i]=np.sqrt(((d_pts[i].point.x-c_pts[i].point.x)**2+
                     (d_pts[i].point.y-c_pts[i].point.y)**2+(d_pts[i].point.z-c_pts[i].point.z)**2))
        d.append(np.mean(diff))
    if(np.argmin(d)<len(G_arr)-1):
        idx=np.argmin(d)
    if(np.abs(d[idx]-d[-1])>0.01):
        # remove inbetween data:
        G_new=G_arr[0:idx+1]+G_arr[-1]
        pickle.dump(G_new, open( file_name, "wb" ) )
def o_files(o_name,l,prefix='r_'):
    ss_names=[]
    for i in range(l):
        ss_names.append(prefix+o_name+'_'+str(i)+'.p')
    return ss_names


if __name__=='__main__':
    l=5
    b_names=o_files('banana',l)
    s_names=o_files('sugar_box',l)
    ss_names=o_files('soft_scrub',l)
    m_names=o_files('mustard',l)
    p_names=o_files('pringles',l)
    names_sd=b_names+s_names+ss_names+m_names+p_names
    for nam in names_sd:
        print nam
        clean_data(nam)
    b_svd_names=o_files('banana',l,'svd_r_')
    s_svd_names=o_files('sugar_box',l,'svd_r_')
    ss_svd_names=o_files('soft_scrub',l,'svd_r_')
    m_svd_names=o_files('mustard_bottle',l,'svd_r_')
    p_svd_names=o_files('pringles',l,'svd_r_')
    names_svd=b_svd_names+s_svd_names+ss_svd_names+m_svd_names+p_svd_names
    print "SVD data!"
    for nam in names_svd:
        print nam
        clean_data(nam)

    #print names
    obj_idx=['banana','sugar_box','soft_scrub','mustard','pringles']
    
