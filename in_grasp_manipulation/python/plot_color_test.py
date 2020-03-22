import numpy as np
import seaborn as sns
import matplotlib as mpl
mpl.rcParams.update({'font.size': 20})
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

sns.set(style='white')

data1 = np.random.rand(10, 1)
data2 = np.random.rand(10, 2)

f, ax = plt.subplots(1, 4)

g='0.9'
sns.boxplot(data=data1, color=g, ax=ax[0])
sns.boxplot(data=data1, color='g', ax=ax[1])
sns.boxplot(data=data2, palette=(g, g), ax=ax[2])
sns.boxplot(data=data1, palette=(g, 'g'), ax=ax[3])

plt.show()
