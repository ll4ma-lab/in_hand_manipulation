import pickle
import rospy
import numpy as np
import matplotlib as mpl
mpl.rcParams.update({'font.size': 20})
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import seaborn as sns
import pandas as pd
safeuiall=['#bae4bc','#7bccc4','#2b8cbe']
safeui=['#bae4bc']

def plot_actual(f_name_,cat_order):
    # check for pickle file, if missing create a new file
    #obj_name=raw_input("Type in object name")
    #if os.path.isfile(f_name_):
    new_d = pickle.load( open( f_name_, "rb" ) )
    #print pic_data
    # gt values:
    gt  = {"mustard_l":0.59,"mustard":0.64,"ss_l":0.45,"ss":0.44,
           "lego":0.68,"cheezit":0.61, "sugar":0.47,"wood":0.62,
           "peg":0.62,"3d-print":0.59}
    #print gt.keys()
    d = {}
    d['object'] = []
    d['mu'] = []
    #d['gt'] = []
    d['method']=[]
    #new_d.keys().sort()
    for j in range(len(new_d.keys())):
        for i in range(len(new_d[new_d.keys()[j]])):
            d['object'].append(new_d.keys()[j])
            d['mu'].append(new_d[new_d.keys()[j]][i])
            d['method'].append('estimation')
            
            d['object'].append(new_d.keys()[j])
            d['method'].append('gt')
            d['mu'].append(gt[new_d.keys()[j]])

    data_frame = pd.DataFrame(data = d)
    #print data_frame
    ax=sns.boxplot(y="mu", x="object",hue = "method",data=data_frame,orient="v",linewidth=2,whis=1.5,width=0.5,showfliers=False,dodge=True, palette = safeuiall,order = cat_order)
    ax.set_ylim(0,0.8)

    plt.show()

def plot_error(f_name_,cat_order):
    new_d = pickle.load( open( f_name_, "rb" ) )

    d = {}
    d['object'] = []
    d['mu'] = []

    gt  = {"mustard_l":0.59,"mustard":0.64,"ss_l":0.45,"ss":0.44,
           "lego":0.68,"cheezit":0.61, "sugar":0.47,"wood":0.62,
           "peg":0.62,"3d-print":0.59}

    #d['gt'] = []
    #d['method']=[]
    #new_d.keys().sort()
    for j in range(len(new_d.keys())):
        for i in range(len(new_d[new_d.keys()[j]])):
            d['object'].append(new_d.keys()[j])
            mu_v = gt[new_d.keys()[j]] - new_d[new_d.keys()[j]][i]
            d['mu'].append(mu_v)
            #d['method'].append('estimation')
            
            d['object'].append("All objects")
            d['mu'].append(mu_v)

            #d['method'].append('gt')
            #d['mu'].append(gt[new_d.keys()[j]])

    data_frame = pd.DataFrame(data = d)
    cat_order.append("All objects")
    #print data_frame
    ax=sns.boxplot(y="mu", x="object",data=data_frame,orient="v",linewidth=2,whis=1.5,width=0.5,showfliers=False,dodge=True, palette = safeui,order = cat_order)

    plt.show()

if __name__=='__main__':
    #f_name_="friction_exp_final.p"
    cat_order =['ss','ss_l','sugar','3d-print','mustard_l','cheezit','wood','mustard','lego']
    f_name_='friction_exp_data.p'
    #cat_order = []
    plot_actual(f_name_,cat_order)
    #plot_error(f_name_,cat_order)
    #print data_frame
    # plot data:
    # data: mu_error, objects
    #print new_d
