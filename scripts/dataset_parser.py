# this file reads and writes to h5py dataset parser.
import h5py
import numpy as np
from data_types import *

class ObjectData:
    ''' This class allows for reading data and writing data to a hdf5 file'''
    def __init__(self):
        self.objects=[]
        print 'hi'
    def read_HDF5(self,file_name):
        self.read_file=file_name
        # read data and return a trial list
        f=h5py.File(file_name,"r")
        num_objects=len(f.keys())
        for i in range(num_objects):
            obj_name=(f.keys())[i]
            obj_group=f[obj_name]

            # Read all data
            dataset=obj_group[obj_group.keys()[0]]
            print dataset.attrs.keys()
            
            # push to objects array
            #self.objects.append(obj)
            

def read_data():
    dataset=ObjectData()
    file_name="../data/rss_data.hdf5"
    dataset.read_HDF5(file_name)

read_data()
