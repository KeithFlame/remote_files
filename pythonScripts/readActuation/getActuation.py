#coding=utf-8
import pandas as pd
import os

import linecache

dfs=pd.DataFrame()
#os.walk(file_path) 深度遍历file_path下的所有子文件夹及文件
params=[]
for root_dir,sub_dir,files in os.walk(r"./onewire/"):
    for file in files:
        if file.endswith(".xlsx"):
            #构造绝对路径
            params2=[]
            file_name = os.path.join(root_dir, file)
            
            serial = file_name.split('O')
            serial = serial[1].split('.')
            serial=serial[0]
            cter = serial[0:3]
            # print(cter)
            serial = int(serial)
            # serial = serial.split('O')
            # print(serial)
            #读取sheet页
            #pd.read_excel(file_path,sheet_name=None).keys()获取excel表格所有的sheet页名称
            # for sheet in  pd.read_excel(file_name,sheet_name=None).keys():
            df=pd.read_excel(file_name)
            data1=df.iloc[53].values
            data2=df.iloc[59].values

            params2.append(int(cter))
            params2.append(serial)
            params2.append(float(data1[3]))
            params2.append(float(data2[3]))
            # print(params2)
        if file.endswith(".log"):
            params2=[]
            file_name = os.path.join(root_dir, file)
            serial = file_name.split('O')
            serial = serial[1].split('.')
            serial=serial[0]
            cter = serial[0:3]
            # print(cter)
            serial = int(serial)
            params2.append(int(cter))
            params2.append(int(serial))
            with open(file_name, 'r') as fp:
                for line in fp.readlines()[54:61]:
                    data1 = line.split(',')
                    if data1[0] == 'value1':
                        params2.append(float(data1[3]))
                    if data1[0] == 'value4':
                        params2.append(float(data1[3]))
            # print(params2)
        params.append(params2)
            # print(data[3])        
                # excel_name=file.replace(".xlsx","")
                # #新增两列用于记录数据所属excel及sheet页，这一步骤感觉很有用，因为后续数据清理的时候，遇到莫名其妙的数据不知道怎么办的话，还可以去源excel表格上看下。
                # df["excel_name"]=excel_name
                # df["sheet_name"]=sheet
                # dfs=pd.concat([dfs,df])
t = params[-1]
for i in range(len(params)-2,-1,-1):
	# print(i)
	if t[1] == params[i][1]:
		# del lists[i]
		params.remove(params[i])
	else:
		t = params[i]
for ele in params:
     print(ele)