# -*- coding: utf-8 -*-
import xlsxwriter as xw
import xlrd
import re
import json
import os
# import ftptest as ftp


NUM_ROW = 63
NUM_COL = 4
# FTP_HOST=''
# FTP_USER=''
# FTP_PASSWORD=''

def get_key(dict, value):
    return [k for k, v in dict.items() if v == value]

def get_open_angle(arm_id,arm_flag):
    if arm_flag == 635:
        res=[0, 0, 0, 0, 0, 0]
        return res

    params={}
    arm_params_file = "../configurationData/arm" + arm_id + "/angleChart.log"
    with open(arm_params_file, 'r') as f:        
        lines = f.readline()
        while lines:
            # print("lines",lines[5])
            line=lines.split('    ')
            params.update({line[0]: float(line[1])})
            lines = f.readline()
            
    # print(params.values())
    # print(params.values().type())

    max_qa = list(params.keys())[list(params.values()).index(max(params.values()))]
    max_open_angle=params[max_qa]
    max_qa=float(max_qa)
    max_1_3_qa=max_qa/3
    max_1_3_open_angle=max_open_angle/3
    max_2_3_qa=max_qa/3*2
    max_2_3_open_angle=max_open_angle/3*2    
    res=[max_1_3_qa, max_1_3_open_angle, max_2_3_qa, max_2_3_open_angle, max_qa, max_open_angle]

    b = [round(i,4) for i in res]
    return b



def getArmName(arm_flag):
    params = {}

	# std::make_pair(2,std::make_pair("SR-ENT-SP0807S", 637)),					//持针钳（简化版）		

	# std::make_pair(4,std::make_pair("SR-ENT-SP0801S", 631)),					//双极弯分离钳（简化版）

	# std::make_pair(6,std::make_pair("SR-ENT-SP0802S", 632)),					//双极弯头抓钳（简化版）

	# std::make_pair(8,std::make_pair("SR-ENT-SP0806S", 636)),					//双极抓钳（简化版）

	# std::make_pair(10,std::make_pair("SR-ENT-SP0808S", 638)),					//组织抓钳（简化版）

	# std::make_pair(12, std::make_pair("SR-ENT-SP0805S", 635)),					//单极电钩（简化版）

	# std::make_pair(14,std::make_pair("SR-ENT-SP0803S", 633))					//单级弯剪（简化版）

    params.update({"631": "双极弯分离钳（简化版）"})
    params.update({"632": "双极弯头抓钳（简化版）"})
    params.update({"633": "单级弯剪（简化版）"})
    params.update({"635": "单极电钩（简化版）"})
    params.update({"636": "双极抓钳（简化版）"})
    params.update({"637": "持针钳（简化版）"})
    params.update({"638": "组织抓钳（简化版）"})

    return params[str(arm_flag)]

def getArmType(arm_flag):
    params = {}
    params.update({"631": 14})
    params.update({"632": 16})
    params.update({"633": 13})
    params.update({"635": 12})
    params.update({"636": 15})
    params.update({"637": 1})
    params.update({"638": 2})

    return params[str(arm_flag)]

def getArmElectronics(arm_flag):
    params = {}
    params.update({"631": 1})
    params.update({"632": 1})
    params.update({"633": 2})
    params.update({"635": 2})
    params.update({"636": 1})
    params.update({"637": 0})
    params.update({"638": 0})

    return params[str(arm_flag)]

def get_arm_id():
    
    with open("../configurationData/filePath.log", 'r') as f:        
        lines = f.readlines()
    arm_id = lines[0]
    return arm_id

def get_arm_params(arm_id):
    arm_params_file = "../configurationData/arm" + arm_id + "/finalPara.log"
    with open(arm_params_file, 'r') as f:        
        lines = f.readlines()
    arm_id_num=int(arm_id)
    while(arm_id_num>1000):
        arm_id_num/=10
        arm_id_num=int(arm_id_num)
    arm_name=getArmName(arm_id_num)
    arm_nelectronics=getArmElectronics(arm_id_num)
    paramsStr = lines[0].split(' ')

    arm_params_file = "../configurationData/arm" + arm_id + "/L1x.log"
    with open(arm_params_file, 'r') as f:        
        lines = f.readlines()
    p_L1x = lines[0].split(' ')
    
    params = {}
    params.update({"name": arm_name})
    params.update({"arm_sn": arm_id})
    params.update({"L1": float(paramsStr[0])})
    params.update({"Lrigid": float(paramsStr[1])})
    params.update({"L2": float(paramsStr[2])})
    params.update({"zeta": float(paramsStr[4])})
    params.update({"K1": float(paramsStr[5])})
    params.update({"K2": float(paramsStr[6])})
    params.update({"Lstem": float(paramsStr[7])})
    params.update({"Gamma1_ini": float(paramsStr[8])})
    params.update({"d": -float(paramsStr[9])})
    params.update({"L1x": float(p_L1x[0])})
    if(arm_nelectronics==1):
        params.update({"isE": 1})
        params.update({"isE_cutting": 0})
        params.update({"isE_coagulation": 1})
        params.update({"is_dual_polar": 1})
    elif(arm_nelectronics==0):
        params.update({"isE": 0})
        params.update({"isE_cutting": 0})
        params.update({"isE_coagulation": 0})
        params.update({"is_dual_polar": 0})
    elif(arm_nelectronics==2):
        params.update({"isE": 1})
        params.update({"isE_cutting": 1})
        params.update({"isE_coagulation": 1})
        params.update({"is_dual_polar": 0})
    if(arm_id_num==635):
        params.update({"is_clamp": 0})
    else:
        params.update({"is_clamp": 1})
    arm_type=getArmType(arm_id_num)
    params.update({"arm_type": arm_type})

    arm_qa_angle=get_open_angle(arm_id,arm_id_num)
    params.update({"Point2": arm_qa_angle[1]})
    params.update({"Value2": arm_qa_angle[0]})
    params.update({"Point3": arm_qa_angle[3]})
    params.update({"Value3": arm_qa_angle[2]})
    params.update({"Point4": arm_qa_angle[5]})
    params.update({"Value4": arm_qa_angle[4]})    
    return params


def get_template_data(filename):
    template_xl = xlrd.open_workbook(filename)
    sheet = template_xl.sheets()[0]
    template_data = []
    for row in range(NUM_ROW):
        row_data = []
        for col in range(NUM_COL):
            value = sheet.cell(row, col).value
            row_data.append(value)
        template_data.append(row_data)
    return template_data


def modify_cell_data(template_data, params):
    result_data = template_data
    template_data[0][3] = params['name']
    template_data[1][3] = params['arm_sn']
    template_data[3][3] = params['is_clamp']
    template_data[4][3] = params['isE_cutting']
    template_data[5][3] = params['isE_coagulation']
    template_data[7][3] = params['isE']
    template_data[8][3] = params['is_dual_polar']
    template_data[10][3] = params['arm_type']
    template_data[14][3] = params['Gamma1_ini']
    template_data[21][3] = params['L1']
    template_data[22][3] = params['Lrigid']
    template_data[23][3] = params['L2']
    template_data[34][3] = params['K1']
    template_data[35][3] = params['K2']
    template_data[27][3] = params['Lstem']
    template_data[26][3] = params['zeta']
    template_data[44][3] = params['d']
    template_data[45][3] = params['L1x']
    template_data[55][3] = params['Point2']
    template_data[56][3] = params['Value2']
    template_data[57][3] = params['Point3']
    template_data[58][3] = params['Value3']
    template_data[59][3] = params['Point4']
    template_data[60][3] = params['Value4']
    return result_data

 
def xw_toExcel(data, file_name):  # xlsxwriter库储存数据到excel
    result_xl = xw.Workbook(file_name)  # 创建工作簿
    worksheet1 = result_xl.add_worksheet("tool_arm")  # 创建子表
    worksheet1.activate()  # 激活表
    i = 1
    # 逐行写入
    for each_row in data:
        row = 'A' + str(i)
        worksheet1.write_row(row, each_row)
        i += 1
    result_xl.close()  # 关闭表
         

def gen_param_xlfile():
    # print(os.getcwd())
    template_data = get_template_data("SR920ToolArmTemplate.xlsx")
    arm_id = get_arm_id()

    params = get_arm_params(arm_id)
    result_data = modify_cell_data(template_data, params)
    arm_params_xlfile = "C:/Users/Administrator/Desktop/烧录专用"+"/SR920ToolArmTemplate_NO"+ arm_id + ".xlsx"
    xw_toExcel(result_data, arm_params_xlfile)
    arm_params_xlfile = "./onewire/SR920ToolArmTemplate_NO"+ arm_id + ".xlsx"
    xw_toExcel(result_data, arm_params_xlfile)



def loadConfig():
    global FTP_HOST
    global FTP_USER
    global FTP_PASSWORD
    with open('ftp_config.json','r') as f:
        data = json.load(f)
        FTP_HOST=data['ftp']['ip']
        FTP_USER=data['ftp']['user']
        FTP_PASSWORD=data['ftp']['password']


if __name__ == '__main__':
    # loadConfig()
    
    os.chdir('./conf/python')
    gen_param_xlfile()

    # arm_id = get_arm_id()

    # params = get_arm_params(arm_id)
    # print(params)
