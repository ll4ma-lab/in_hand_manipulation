# This file takes mean, variance and plots them as box plot using seaborn

import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42

import seaborn as sns
#plt.style.use('seaborn-white')
#safeui = ['#fef0d9','#fdcc8a','#fc8d59','#d7301f']
safeuiall=['#e41a1c','#377eb8','#4daf4a','#984ea3','#ff7f00']
twoui=['#a6611a','#dfc27d','#f5f5f5','#80cdc1','#018571']
safeui=['#fdcc8a']
#sns.color_palette(safeui)
#sns.palplot(sns.color_palette(safeui))
def load_csv():
    #errors=pd.read_csv('desired_reached_only_closed_loop.csv')
    errors=pd.read_csv('weight_plot.csv')
    return errors


def box_plot(data_):
    #sns.set_context("paper")
    plt.figure(figsize=(27,10))

    
    sns.set(font_scale=1.45)
    sns.set_style("whitegrid")

    #ax=sns.boxplot(y="E-pos", x="Weights-k2",hue="k2",data=data_,orient="v",linewidth=1,palette=twoui)
    ax=sns.swarmplot(y="E-pos", x="Weights-k2",data=data_,orient="v",hue="k2",linewidth=1,palette=safeuiall)

    ax.set(xlabel='Weights[k1,k3]', ylabel='Position Error')

    ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.07),ncol=5, fancybox=False, shadow=False)
    #ax.legend().set_visible(False)

    plt.show()
    #plt.xlim(0,100)
    #plt.savefig(x_pos+'.eps', bbox_inches='tight')

    #plt.figure(figsize=(7,10))

    '''
    sns.set(font_scale=1.45)

    sns.set_style("whitegrid")

    ay=sns.boxplot(y="Object", x=x_orient,data=data_,orient="h",whis=20.0,palette=safeui,linewidth=1) 
    ay=sns.swarmplot(y="Object", x=x_orient,data=data_,orient="h",hue="Trajectory",palette=safeuiall,linewidth=1)     
    ay.set(xlabel='Orientation error', ylabel='Object')
    #ay.set(xlabel='Orientation error', ylabel='Object')
    ay.legend(loc='upper center', bbox_to_anchor=(0.5, 1.07),
          ncol=5, fancybox=True, shadow=True)
    plt.xlim(0,3.14)

    plt.savefig(x_orient+'.eps', bbox_inches='tight')
    '''
if __name__=='__main__':
    data=load_csv()
    box_plot(data)
    #box_plot_rs(data)
    #success_plot(data)
    #orient_plot(data)
    #box_plot(data,'Position Error','Normalized Quaternion Error')
    #box_plot(data,'Normalized Position Error','Angle-axis Error')
    #box_d_s_plot(data)
    
