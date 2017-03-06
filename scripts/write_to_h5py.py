# This file was used to store all the previously generated data into h5py.
import h5py
import numpy as np

f=h5py.File("rss_data.hdf5","w")
dset=f.create_dataset("in_grasp_dataset",(),dtype='i')

# 
