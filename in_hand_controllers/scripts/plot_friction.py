import pickle
import numpy as np
import matplotlib as mpl
#import pylab as plt
#import plotly.plotly as py
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

#mpl.rcParams['axes.titlesize']='large'
#mpl.rcParams['axes.labelsize']='large'

#plt.rcParams.update({'axes.titlesize': 'large'})
import seaborn as sns
sns.set(style="whitegrid")
mpl.rcParams.update({'font.size': 20})
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

#safeuiall=['#e41a1c','#377eb8','#4daf4a','#984ea3','#ff7f00','#d7301f','#fef0d9']
safeuiall=['#ffffff','#f0f9e8','#bae4bc','#7bccc4','#2b8cbe']
#safeuiall=['#fef0d9','#d7301f']
#safeuiall=['#fdae61','#fdae61','#fdae61','#abd9e9','#2c7bb6','#d7301f']

import pandas as pd


if __name__=='__main__':
    f_name_="friction_exp.p"
    p_data = pickle.load( open( f_name_, "rb" ) )
    objs=['pitcher','plane','plane','wood','sugar-box','cheezit','sugar-box','drill-box','soft-scrub']
    #objs=['lego','plane','wood','sugar-box','drill-box','soft-scrub','cheezit','pringles']
    obj_names=[]
    mu=[]
    for i in range(len(objs)):
        for j in range(len(p_data[objs[i]])):
            if(p_data[objs[i]][j]>0.15):
                mu.append((p_data[objs[i]][j]))
                obj_names.append(objs[i])

    ind=range(len(mu))
    pd_data=pd.DataFrame({'mu':mu,'object':obj_names},index=ind)
    #print p_data['drill-box']
    print pd_data
    #print pd_data.dtypes
    ax=sns.boxplot(y="mu", x="object",data=pd_data,orient="v",
                   linewidth=2,whis=10,width=0.5,showfliers=False)
    ax.set_ylabel('Coefficient of Friction ($\mu$)',fontsize='20',labelpad=0)
    ax.set_xlabel('Object',fontsize='20',labelpad=10)
    ax.tick_params(axis='x', labelsize='20')
    ax.tick_params(axis='y', labelsize='20')

    plt.show()
