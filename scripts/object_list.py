# this class creates an object instance with parameters to help with scripting
import numpy as np
import csv
import copy
#import PyKDL

class ObjectReachList:
    '''
    Initialized with initial grasp and offset and desired thumb pose to get goal pose of object
    '''
    def __init__(self,csv_file,folder,tr_start=0):
        self.arr=[]
        self.file=csv_file
        self.folder=folder
        i=0
        self.lines=[]
        q=tr_start
        prev_name="kiq" # random name
        # read file 
        with open(csv_file,'rb') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='"')
            for row in spamreader:
                line=row
                self.lines.append(line)
                offset=np.array(line[1].split(','))
                name=line[0]
                grasp=np.array(map(float, line[2].split(',')))
                fing=np.array(map(int, line[3].split(',')))
                des=np.array(map(float, line[10:]))
                #des=[]
                initial_pose=np.array(map(float, line[4:10]))
                if name==prev_name:
                    q+=1                  
                else:
                    prev_name=name
                    q=tr_start
                f_name=name+'_'+str(q)                 

                self.arr.append(ObjectModel(name,grasp,offset,des,fing,i,f_name,folder,initial_pose))
                i+=1
        print "Read "+str(i)+" objects"
            
    def write_sol_from_tjo(self,file_w):
        i=0
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for row in self.lines:
                list_sol=map(str,self.arr[i].sol)
                data=list(row)+list_sol
                i+=1
                writer.writerow(data)
    def write_sol_from_tjo(self,file_w):
        i=0
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for i in range(len(self.arr)):
                obj=self.arr[i]
                row=[obj.name,','.join(map(str,obj.offset)),','.join(map(str,obj.grasp)),','.join(map(str,obj.fing))]+list(obj.initial_pose)+list(obj.sol)
                data=row
                i+=1
                writer.writerow(data)

    def write_weights(self,file_w):
        i=0
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for row in self.lines:
                list_sol=map(str,self.arr[i].sol)
                data=list(row)+list_sol
                i+=1
                writer.writerow(data)


class ObjectModel:
    def __init__(self,name,grasp,offset,x_des,fing,i,f_name,folder,initial_pose,solution=[],reached=[],weights=[],trial=0):
        self.grasp=grasp
        self.offset=offset
        self.name=name
        self.des=x_des
        self.fing=fing
        self.row=i
        self.f_name=f_name
        self.initial_pose=initial_pose
        self.sol=solution
        self.folder=folder
        self.weights=weights
        self.trial=trial
        self.reached=reached

class TrialList:
    '''Reading trial csv file'''
    def __init__(self,csv_file,folder,n_trials):
        self.arr=[]
        f_name=folder+csv_file
        with open(f_name,'r') as csvfile:
            csvreader=csv.reader(csvfile,delimiter=',',quotechar='"')
            for row in csvreader:
                line=row
                name=line[0]
                f_name=line[1]
                grasp=np.array(map(float, line[2].split(',')))
                weights=np.array(map(float, line[4].split(',')))
                offset=np.array(line[3].split(','))                               
                initial_pose=np.array(map(float, line[5:5+6]))
                des=np.array(map(float, line[11:11+6]))
                solution=np.array(map(float, line[17:17+6]))
                trial_no=line[23]
                folder=folder
                fing=[]
                line_length=len(line)
                if(line_length>24):
                    reached=map(str,line[24:line_length])
                else:
                    reached=[]
                self.arr.append(ObjectModel(name,grasp,offset,des,fing,csvreader.line_num,f_name,folder,initial_pose,solution,reached,weights,trial_no))

    def write_reached_pose(self,file_w):

        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for i in range(len(self.arr)):
                obj=self.arr[i]
                row=[obj.name,obj.f_name,','.join(map(str,obj.grasp)),','.join(map(str,obj.offset)),','.join(map(str,obj.weights))]+list(obj.initial_pose)+list(obj.des)+list(obj.reached)
                writer.writerow(row)

class TrajList:
    ''' Reading  solution.csv file'''
    def __init__(self,csv_file,folder,tr_start=0):
        self.arr=[]
        self.file=csv_file
        self.folder=folder
        self.lines=[]
        with open(csv_file,'r') as csvfile:
            csvreader= csv.reader(csvfile, delimiter=',', quotechar='"')
            for row in csvreader:
                line=row
                name=line[0]
                f_name=line[1]
                offset=np.array(line[3].split(','))
                grasp=np.array(map(float, line[2].split(',')))
                weights=np.array(map(float, line[4].split(',')))
                initial_pose=np.array(map(float, line[5:5+6]))
                des=np.array(map(float, line[11:11+6]))
                solution=np.array(map(float, line[17:17+6]))
                fing=[0,1,3]
                folder=folder
                self.arr.append(ObjectModel(name,grasp,offset,des,fing,csvreader.line_num,f_name,folder,initial_pose,solution,weights=weights))

    def write_trial_list(self,n_trials,f_name):
        ''' create csv file with name f_name and n_trials per trajectory'''
        file_w=self.arr[0].folder+f_name
        trial_arr=range(n_trials)
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for i in range(len(self.arr)):
                obj=self.arr[i]
                row=[obj.name,obj.f_name,','.join(map(str,obj.grasp)),','.join(map(str,obj.offset)),','.join(map(str,obj.weights))]+list(obj.initial_pose)+list(obj.des)+list(obj.sol)
                for k in trial_arr:
                    data=row+[k]
                    #write to csv file
                    writer.writerow(data)

class ObjectList:
    def __init__(self,csv_file,folder,tr_start=0):
        self.arr=[]
        self.file=csv_file
        self.folder=folder
        i=0
        self.lines=[]
        q=tr_start
        prev_name="kiq"
        # read file 
        with open(csv_file,'rb') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar='"')
            for row in spamreader:
                line=row
                self.lines.append(line)
                offset=np.array(line[1].split(','))
                name=line[0]
                grasp=np.array(map(float, line[2].split(',')))
                fing=np.array(map(int, line[3].split(',')))
                des=np.array(map(float, line[10:16]))
                #initial_pose=[]
                initial_pose=np.array(map(float,line[4:10]))
                if name==prev_name:
                    q+=1                  
                else:
                    prev_name=name
                    q=tr_start
                f_name=name+'_'+str(q)                 
                
                self.arr.append(ObjectModel(name,grasp,offset,des,fing,i,f_name,folder,initial_pose))
                i+=1
        print "Read "+str(i)+" objects"
    def get_weights(self,file_name):
        with open(file_name,'r') as csvfile:
            row_count = sum(1 for row in csvfile)  
            initial_arr=copy.deepcopy(self.arr)
            traj_count=len(initial_arr)
            self.arr=[]
            for i in range(row_count):
                new_set=copy.deepcopy(initial_arr)
                for k in range(len(new_set)):
                    new_set[k].row=k+(i)*traj_count
                    new_set[k].f_name=new_set[k].f_name+'_'+str(new_set[k].row)
                self.arr.extend(copy.deepcopy(new_set))
            csvfile.seek(0)
            csvreader = csv.reader(csvfile, delimiter=',', quotechar='"')
            w=[]
            for row in csvreader:
                k1=float(row[0])
                k2=float(row[1])
                k3=float(row[2])
                k4=float(row[3])
                max_iter=int(row[4])
                weights=np.array([k1,k2,k3,k4,max_iter])
                for i in range(traj_count):
                    self.arr[i+(csvreader.line_num-1)*(len(initial_arr))].weights=weights

    def add_weights(self,file_name):
        with open(file_name,'r') as csvfile:
            csvreader = csv.reader(csvfile, delimiter=',', quotechar='"')
            for row in csvreader:
                k1=float(row[0])
                k2=float(row[1])
                k3=float(row[2])
                k4=float(row[3])
                weights=np.array([k1,k2,k3,k4])
                self.arr[csvreader.line_num-1].weights=weights
        


    def write_sol(self,file_w):
        i=0
        with open(file_w,'r+') as f:
            writer= csv.writer(f)
            for row in self.lines:
                list_sol=map(str,self.arr[i].sol)
                data=list(row)+list_sol
                i+=1
                writer.writerow(data)

    def write_sol_from_tjo(self,file_w):
        i=0
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for i in range(len(self.arr)):
                obj=self.arr[i]
                row=[obj.name,obj.f_name,','.join(map(str,obj.grasp)),','.join(map(str,obj.offset)),','.join(map(str,obj.weights))]+list(obj.initial_pose)+list(obj.des)
                list_sol=map(str,self.arr[i].sol)
                data=row+list_sol
                i+=1
                writer.writerow(data)
    def write_weights(self,file_w):
        i=0
        with open(file_w,'w') as f:
            writer= csv.writer(f)
            for i in range(len(self.arr)):
                obj=self.arr[i]
                data=[obj.name]+list(obj.weights)+list(obj.des)+obj.sol
                writer.writerow(data)
