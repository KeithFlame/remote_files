# -*- coding: utf-8 -*-
import numpy as np

def getOpenAngle(file_id):
    params=[]
    arm_params_file = file_id
    with open(arm_params_file, 'r') as f:        
        # lines = f.readline()
        lines = f.readline()
        
        while lines:
            
            # print("\n")
            sub_params=[]
            line=lines.split('\t')
            # print(line)
            i=0
            for ele in line:
                sub_params.append(ele)
                i+=1
            del sub_params[0 : 2]
            sub_params.pop()
            params.append(sub_params)
            lines = f.readline()
            matrix_num=(i-3)/12
    print(params)
    return (params,matrix_num)

def calcMatrix(params,flag,matrix_num=2):
    ang_dis=[]
    matrixA=np.asmatrix(np.zeros([4,4]))
    matrixB=np.asmatrix(np.zeros([4,4]))
    # print("params:",type(params))
    # params=params[1]
    # print(params)
    for ele in params:
        # print(ele)
        # print("\n")
        matrixA[0,0]=float(ele[3])
        matrixA[0,1]=float(ele[4])
        matrixA[0,2]=float(ele[5])
        matrixA[1,0]=float(ele[6])
        matrixA[1,1]=float(ele[7])
        matrixA[1,2]=float(ele[8])
        matrixA[2,0]=float(ele[9])
        matrixA[2,1]=float(ele[10])
        matrixA[2,2]=float(ele[11])
        matrixA[0,3]=float(ele[0])
        matrixA[1,3]=float(ele[1])
        matrixA[2,3]=float(ele[2])
        matrixA[3,3]=1

        matrixB[0,0]=float(ele[15])
        matrixB[0,1]=float(ele[16])
        matrixB[0,2]=float(ele[17])
        matrixB[1,0]=float(ele[18])
        matrixB[1,1]=float(ele[19])
        matrixB[1,2]=float(ele[20])
        matrixB[2,0]=float(ele[21])
        matrixB[2,1]=float(ele[22])
        matrixB[2,2]=float(ele[23])
        matrixB[0,3]=float(ele[12])
        matrixB[1,3]=float(ele[13])
        matrixB[2,3]=float(ele[14])
        matrixB[3,3]=1
        if(flag):
            c=np.dot(matrixA.I,matrixB)
        else:
            c=np.dot(matrixB.I,matrixA)
        sub_ang_dis=[]

        ang=np.arccos((c[0,0]+c[1,1]+c[2,2]-1)/2)
        ang=ang/np.pi*180
        sub_ang_dis.append("rotation: ")
        sub_ang_dis.append(ang)
        sub_ang_dis.append(",  position: ")
        sub_ang_dis.append(c[0,3])
        sub_ang_dis.append(c[1,3])
        sub_ang_dis.append(c[2,3])
        sub_ang_dis.append("], dis=: ")
        x=[c[0,3],c[1,3], c[2,3]]
        sub_ang_dis.append(np.linalg.norm(x,ord=2))
        ang_dis.append(sub_ang_dis)
    return ang_dis

def showIt(name,flag):
    file_id=name
    (params,m)=getOpenAngle(file_id)
    ang_dis=calcMatrix(params,flag)
    for ele in ang_dis:
        print(ele)
        print("\n")
if __name__ == "__main__":
    name="tip_D2_2.raw"
    flag=1
    showIt(name,flag)

