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
mpl.rcParams['axes.unicode_minus'] = False

safeuiall=['#FFFFFF','#7bccc4','#2b8cbe',"#CC0000"]
sep_tags=['#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#FFFFFF',
          '#7bccc4']

safeui=['#FFFFFF','#000000']
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

def plot_actual(f_name_,cat_order):
    # check for pickle file, if missing create a new file
    #obj_name=raw_input("Type in object name")
    #if os.path.isfile(f_name_):
    new_d = pickle.load( open( f_name_, "rb" ) )
    #print pic_data
    # gt values:
    gt  = {"mustard_l":0.59,"mustard":0.64,"ss_l":0.45,"ss":0.44,
           "lego":0.68,"cheezit":0.61, "sugar":0.47,"wood":0.62,
           "peg":0.62,"3d-print":0.59,"peg":0.67}
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
            d['method'].append('gt')
            d['mu'].append(gt[new_d.keys()[j]])

            d['object'].append(new_d.keys()[j])
            d['mu'].append(new_d[new_d.keys()[j]][i])
            d['method'].append('estimation')
            
     
    data_frame = pd.DataFrame(data = d)
    data_frame1 = data_frame[data_frame["method"]=='estimation']

    data_frame2 = data_frame[data_frame["method"]=='gt']

    plt.figure(figsize=(10,4))
    
    
    sns.set(font_scale=1.15)
    sns.set_style("white")

    #print data_frame
    ax=sns.boxplot(y="mu", x="object",hue = "method",data=data_frame1,orient="v",linewidth=1.25,whis=1.5,width=0.6,showfliers=False,
                   dodge=False, palette = safeui,order = cat_order)

    plt.setp(ax.lines, color="black")

    ax=sns.pointplot(y="mu", x="object",
                     hue ="method", data=data_frame2, orient="v",linewidth=2,width=0.5,
                     dodge=False, palette = safeuiall[3:],order = cat_order, join = False,markers=["x"],scale=1.0)


    ax.set_ylim(0.35,0.8)
    #ax.set_ylim(0.0,0.8)
    ax.get_legend().remove()
    ax.set(xlabel='Object Name',ylabel='Friction Coefficient ($\mu$)')

    labels = ax.get_xticklabels()
    # map labels:
    ycb_names={"ss":"bleach\ncleanser",
               "ss_l":"bleach\ncleanser\nlabel",
               "sugar":"box of\nsugar",
               "3d-print":"3D\nprinted\nobject",
               "mustard_l":"mustard\ncontainer\nlabel",
               "cheezit":"cracker\nbox",
               "wood":"wood\nblock",
               "mustard":"mustard\ncontainer",
               "peg":"9 hole\n peg test",
               "lego":"Lego\nDublo"}
    for i in range(len(labels)):
        labels[i].set_text(ycb_names[labels[i].get_text()])
    # set labels:
    ax.set_xticklabels(labels)
    ax.grid(b=True, which='minor', color='0.75', linewidth=0.3,axis='y')
    ax.grid(b=True, which='major', color='0.65', linewidth=0.6,axis='y')

    #ax.get_yaxis().set_minor_locator(mpl.ticker.AutoMinorLocator())

    #ax.set_yticks(np.arange(0.0, 0.8,0.1), minor=True)
    #plt.yticks(np.arange(0.0, 0.8,0.1),minor=True)
    ax.get_yaxis().set_minor_locator(mpl.ticker.MultipleLocator(0.05))

    # iterate over boxes
    for i,box in enumerate(ax.artists):
        box.set_edgecolor('black')
        #box.set_facecolor('white')
        
        # iterate over whiskers and median lines
        #for j in range(6*i,6*(i+1)):
        #    ax.lines[j].set_color('black')

    plt.savefig('/home/bala/friction_zoom_plot.eps', bbox_inches='tight')
    #plt.savefig('/home/bala/friction_plot.eps', bbox_inches='tight')
    #plt.show()

def plot_error(f_name_,cat_order):
    new_d = pickle.load( open( f_name_, "rb" ) )

    d = {}
    d['object'] = []
    d['mu'] = []

    gt  = {"mustard_l":0.59,"mustard":0.64,"ss_l":0.45,"ss":0.44,
           "lego":0.68,"cheezit":0.61, "sugar":0.47,"wood":0.62,
           "peg":0.67,"3d-print":0.59}

    #d['gt'] = []
    d['method']=[]
    #new_d.keys().sort()
    for j in range(len(new_d.keys())):
        for i in range(len(new_d[new_d.keys()[j]])):
            d['object'].append(new_d.keys()[j])
            #mu_v = np.rad2deg(np.arctan((gt[new_d.keys()[j]] - new_d[new_d.keys()[j]][i])))
            mu_v =(gt[new_d.keys()[j]] - new_d[new_d.keys()[j]][i])
            d['mu'].append(mu_v)
            d['method'].append('estimation')
            
            d['object'].append("All objects")
            d['mu'].append(mu_v)
            d['method'].append('estimation')

            #d['method'].append('gt')
            #d['mu'].append(gt[new_d.keys()[j]])

    
    data_frame = pd.DataFrame(data = d)
    cat_order.append("All objects")
    mu_all= np.array(data_frame[data_frame['object']=='All objects']['mu'])
    print np.min(mu_all), np.max(mu_all),np.median(mu_all)
    #print data_frame
    plt.figure(figsize=(10,4))
    
    
    sns.set(font_scale=1.15)
    sns.set_style("white")
    #ax.axhline(0.0, linestyle='--', linewidth = 1.0,color=safeuiall[3])
        
    ax=sns.boxplot(y="mu", x="object",data=data_frame,orient="v",
                   linewidth=1.25,whis=1.5,width=0.4,showfliers=False,dodge=False,
                   palette = sep_tags,order = cat_order)



    # iterate over boxes
    plt.setp(ax.lines, color="black")

    for i,box in enumerate(ax.artists):
        box.set_edgecolor('black')


    labels = ax.get_xticklabels()
    # map labels:
    ycb_names={"ss":"bleach\ncleanser",
               "ss_l":"bleach\ncleanser\nlabel",
               "sugar":"box of\nsugar",
               "3d-print":"3D\nprinted\nobject",
               "mustard_l":"mustard\ncontainer\nlabel",
               "cheezit":"cracker\nbox",
               "wood":"wood\nblock",
               "mustard":"mustard\ncontainer",
               "peg":"9 hole\n peg test",
               "lego":"Lego\nDublo",
               "All objects":"All\nobjects"}
    for i in range(len(labels)):
        labels[i].set_text(ycb_names[labels[i].get_text()])
    # set labels:
    ax.set_xticklabels(labels)

    ax.grid(b=True, which='minor', color='0.65', linewidth=0.3,axis='y')
    ax.grid(b=True, which='major', color='0.45', linewidth=0.6,axis='y')

    #ax.get_yaxis().set_minor_locator(mpl.ticker.AutoMinorLocator())

    #ax.set_yticks(np.arange(0.0, 0.8,0.1), minor=True)
    #plt.yticks(np.arange(0.0, 0.8,0.1),minor=True)
    ax.get_yaxis().set_minor_locator(mpl.ticker.MultipleLocator(0.025))

    a = ax.get_ygridlines()
    b = a[4]
    b.set_color(safeuiall[3])
    b.set_linewidth(1.5)
    b.set_alpha(1.0)
    #ax.get_yaxis().set_minor_locator(mpl.ticker.AutoMinorLocator())
    #ax.set_yticks(np.arange(0.0, 0.8,0.1), minor=True)
    #plt.yticks(np.arange(0.0, 0.8,0.1),minor=True)
    #ax.get_legend().remove()
    ax.set(xlabel='Object Name',ylabel='Friction Estimation Error ($\mu$)')
    plt.savefig('/home/bala/friction_error.eps', bbox_inches='tight')

    #plt.show()

if __name__=='__main__':
    #f_name_="friction_exp_final.p"
    cat_order =['ss','ss_l','sugar','3d-print','mustard_l','cheezit','wood','mustard','peg','lego']
    f_name_='friction_exp_final_v2.p'
    #cat_order = []
    plot_actual(f_name_,cat_order)
    #plot_error(f_name_,cat_order)
    #new_d = pickle.load( open( f_name_, "rb" ) )

    #f_name_='friction_exp_final.p'
    #peg_d = pickle.load( open( f_name_, "rb" ) )
    #mu = []
    #for i in range(len(peg_d['peg'])):
    #    mu.append(peg_d['peg'][i]['mu'])
    #new_d['peg'] = mu
    #pickle.dump(new_d,open('friction_exp_final_v2.p',"wb"))
    #print len(new_d['peg'])
              #new_d['peg'][0].keys()
    #print data_frame
    # plot data:
    # data: mu_error, objects
    #print new_d
