import numpy as np
import pandas as pd

class pdAPI:
    def create_pd_frame(self,column_labels):
        # Create a pd file with column_labels and save to f_name
        df = pd.DataFrame(columns=column_labels)
        return df
    def write_pd_data(self,df,column_data):
        df_new=pd.DataFrame(column_data,columns=list(df.columns))
        df=df.append(df_new)
        return df
    def write_pd_file(self,f_name,df):
        df.to_csv(f_name,index=False)
    def read_pd(self,f_name):
        df=pd.read_csv(f_name)
        return df
    
'''
if __name__=='__main__':
    pd_api=pdAPI()
    df=pd_api.read_pd('foo.csv')
   
    df=pd_api.create_pd_frame(['object','method'])
    data=[['apple','obj']]
    df=pd_api.write_pd_data(df,data)

'''
