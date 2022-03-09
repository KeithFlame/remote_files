from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, PageBreak, Image, Table, TableStyle, Spacer, PageTemplate, Frame
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.pdfgen import canvas 
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.pagesizes import LETTER
from reportlab.lib.units import cm
import os
import xlsxwriter as xw
import xlrd
import re

pdfmetrics.registerFont(TTFont('simhei', 'simhei.ttf'))  #注册字体
pdfmetrics.registerFont(TTFont('simkai', 'simkai.ttf'))  #注册字体
pdfmetrics.registerFont(TTFont('simsun', 'simsun.ttc'))  #注册字体
NUM_ROW = 63
NUM_COL = 4

class FooterCanvas(canvas.Canvas):

    def __init__(self, *args, **kwargs):
        canvas.Canvas.__init__(self, *args, **kwargs)
        self.pages = []

    def showPage(self):
        self.pages.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        page_count = len(self.pages)
        for page in self.pages:
            self.__dict__.update(page)
            self.draw_canvas(page_count)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_canvas(self, page_count):
        page = "BSR-FORM-4431(V1)                            %s / %s                           机密文件，严格保密" % (self._pageNumber, page_count)
        x = 550
        self.saveState()
        self.setStrokeColorRGB(0, 0, 0)
        self.setLineWidth(0.5)
        # self.line(66, 78, LETTER[0] - 66, 78)
        self.setFont('simsun', 10)
        self.drawString(LETTER[0]-x, 65, page)
        self.restoreState()



class FooterCanvas_old(canvas.Canvas):

    def __init__(self, *args, **kwargs):
        canvas.Canvas.__init__(self, *args, **kwargs)
        self.pages = []

    def showPage(self):
        self.pages.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        page_count = len(self.pages)
        for page in self.pages:
            self.__dict__.update(page)
            self.draw_canvas(page_count)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_canvas(self, page_count):
        page = "BSR-P031-R02(A)                              %s / %s                           机密文件，严格保密" % (self._pageNumber, page_count)        
        x = 550
        self.saveState()
        self.setStrokeColorRGB(0, 0, 0)
        self.setLineWidth(0.5)
        # self.line(66, 78, LETTER[0] - 66, 78)
        self.setFont('simsun', 10)
        self.drawString(LETTER[0]-x, 65, page)
        self.restoreState()


def get_arm_id(is_new_template=1):
    if(is_new_template):
        with open("../configurationData/filePath.log", 'r') as f:        
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        arm_id = line
        return arm_id
    else:
        with open("arm_serial.log", 'r') as f:        
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        with open(line, 'r') as f:        
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        arm_id = line
        return arm_id



def get_arm_PID(arm_id,is_new_template):
    if(is_new_template):
        with open("../configurationData/arm"+arm_id+"/production_number.log", 'r') as f:        
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        arm_PID= line
        return arm_PID
    else:
        with open("arm_serial.log", 'r') as f:        
            lines = f.readlines()
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        with open(line, 'r') as f:        
            lines = f.readlines()
        line = lines[0]
        line = line.replace("\n", "")
        arm_PID= line
        return arm_PID

def get_single_line_result(fname):
    with open(fname, 'r') as f:        
        lines = f.readlines()
    result = lines
    return result

def get_cover_tbl():
    arm_id = get_arm_id(is_new_template)
    arm_PID= get_arm_PID(arm_id,is_new_template)
    cover_tbl = [['工    序：', '300 ', '编    号：', 'B']]
    cover_tbl.append(['产品/部件名称：', '3D电子内窥镜', '检 验 员：', ''])
    cover_tbl.append(['产品/部件型号：', 'SR-ENT-SP1001', '测试日期：', ''])
    cover_tbl.append(['序 列 号：', arm_id, '复 核 人：', ''])
    cover_tbl.append(['生产批号：', arm_PID, '复核日期：', ''])
    cover_tbl.append(['测试设备名称：', '手术工具性能测试工装', '设备编号：', 'GJ-056'])
    cover_tbl.append(['检验依据：', '《手术工具产品成品检验规程》', 'BSR-SOP-3315', ''])
    t = Table(cover_tbl, colWidths=[90, 180, 60, 120])
    t.setStyle(TableStyle(
        [('INNERGRID', (1, 1), (-1,-1), 0.25, colors.black),
        ('BOX', (0,0), (-1,-1), 1, colors.black),
        ('ALIGN', (0,0), (0,-1), 'RIGHT'),
        ('ALIGN', (2,0), (2,-1), 'RIGHT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('SPAN', (1, -1), (-1,-1)),
        ('FONT', (0,0), (-1,-1), 'simsun')]
        ))  
    return t 

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

def get_cell_data(template_data):
    params={}
    params['name']=template_data[0][3] 
    params['Gamma1_ini']=template_data[14][3]
    params['L1']=template_data[21][3]
    params['Lrigid']=template_data[22][3]
    params['L2']=template_data[23][3]
    params['K1']=template_data[34][3]
    params['K2']=template_data[35][3]
    params['Lstem']=template_data[27][3]
    params['zeta']=template_data[26][3]
    params['d']=template_data[44][3]
    params['L1x']=template_data[45][3]
    params['Point4']=template_data[59][3]
    params['Value4']=template_data[60][3]
    return params

def get_arm_params():
    # print(os.getcwd())
    template_data = get_template_data("SR920ToolArmTemplate.xlsx")
    arm_id = get_arm_id()

    params = get_cell_data(arm_id)
    return params


def get_comment_sense_test(flag=1):
    if(flag==1):
        summary_result = [['检验项目', '测试细项', '接受标准', '测试结果', '判定']]

        result_mov_ability = ['周向弯转', '周向弯转测试']
        P = Paragraph("进行测试，均可往周\n向任意方向弯转。", styles['Normal_ch'])
        result_mov_ability.append(P)
        result_mov_ability.append('满足要求')
        result_mov_ability.append('合格')
        summary_result.append(result_mov_ability)

    elif(flag==2):
        summary_result=[]
        arm_id = get_arm_id()
        joint_limit_result_file = "../configurationData/arm" + arm_id + "/limit_joint_data.log"
        joint_limit_ok = get_single_line_result(joint_limit_result_file)
        joint_limit=joint_limit_ok[0].split('    ')
        print(joint_limit[0])
        result_joint_limit = ['运动范围', '弯转角度']
        P = Paragraph("臂体第一构节最大弯转90°±2°;\n臂体第二构节最大弯转120°±2°。", styles['Normal_ch'])
        result_joint_limit.append(P)
        tem=joint_limit[0]+"°, \n"+joint_limit[1]+"°"
        P = Paragraph(tem, styles['Normalcell_ch_underline'])
        result_joint_limit.append(P)
        if abs(float(joint_limit[0])-90)<2 and abs(float(joint_limit[1])-120)<2 :
            result_joint_limit.append('合格')
        else:
            result_joint_limit.append('不合格')
        summary_result.append(result_joint_limit)

    t = Table(summary_result, colWidths=[70, 70, 190, 70, 50])
    t.setStyle(TableStyle(
    [('INNERGRID', (0, 0), (-1,-1), 0.5, colors.black),
    ('BOX', (0,0), (-1,-1), 0.5, colors.black),
    ('ALIGN', (0,0), (-1,-1), 'CENTER'),
    ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ('SPAN',(0,1),(0,1)),
    ('SPAN',(0,3),(0,4)),
    ('FONT', (0,0), (-1,-1), 'simsun')]
    )) 
    return t
def get_kine_calib_result(arm_id):
    filename="../testEditExcel/onewire/SR920ToolArmTemplate_NO"+arm_id+".xlsx"
    template_data=get_template_data(filename)
    params=get_cell_data(template_data)
    L1 = params['L1']
    Lr = params['Lrigid']
    L2 = params['L2']
    Lg = 15
    K1 = params['K1']
    K2 = params['K2']
    Lstem = params['Lstem']
    Gamma_g = 0
    Gamma_a = params['Gamma1_ini']
    zeta=params['zeta']

    summary_result = [['L1', 'Lr', 'L2', 'Lg', 'Lstem',  'K1', 'K2','zeta','Gamma_a','Gamma_g']]
    summary_result.append([L1, Lr, L2, Lg,Lstem, K1, K2,zeta, Gamma_a,  Gamma_g])

    t = Table(summary_result, colWidths=50)
    t.setStyle(TableStyle(
        [('INNERGRID', (0, 0), (-1,-1), 0.25, colors.black),
        ('BOX', (0,0), (-1,-1), 1, colors.black),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE')]
        ))    
    return t


def get_fat_result(arm_id):

    

    
    precision_file = "../configurationData/arm" + arm_id + "/arm_precision_data.log"
    space_limit_result_file = "../configurationData/arm" + arm_id + "/limit_pose_data.log"
    

    # max_open_angle_file = "../configurationData/arm" + arm_id + "/angleChart.log"
    # limit_force_file="../configurationData/arm" + arm_id + "/z_force_data.log"

    
    precision_ok = get_single_line_result(precision_file)
    space_limit_result = get_single_line_result(space_limit_result_file)
    summary_result = [['检验项目', '测试细项', '接受标准', '测试结果', '判定']]

    result_mov_ability = ['周向弯转', '周向弯转测试']
    P = Paragraph("进行测试，均可往周\n向任意方向弯转。", styles['Normal_ch'])
    result_mov_ability.append(P)
    if 1:
        result_mov_ability.append('满足要求')
        result_mov_ability.append('合格')
    else:
        result_mov_ability.append('不满足要求')
        result_mov_ability.append('不合格')
    summary_result.append(result_mov_ability)


    precision=precision_ok[0].split('    ')
    result_mov_ability = ['运动精度', '位置准确度']
    P = Paragraph("≤2mm", styles['Normal_ch'])
    result_mov_ability.append(P)
    if float(precision[0])<2:
        result_mov_ability.append('满足要求')
        result_mov_ability.append('合格')
    else:
        result_mov_ability.append('不满足要求')
        result_mov_ability.append('不合格')
    summary_result.append(result_mov_ability)






    t = Table(summary_result, colWidths=[70, 70, 190, 70, 50])
    t.setStyle(TableStyle(
        [('INNERGRID', (0, 0), (-1,-1), 0.5, colors.black),
        ('BOX', (0,0), (-1,-1), 0.5, colors.black),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('SPAN',(0,1),(0,1)),
        ('SPAN',(0,3),(0,4)),
        ('FONT', (0,0), (-1,-1), 'simsun')]
        )) 
    final_ok =  0
    return t, final_ok


# 获得模板表格
styles = getSampleStyleSheet()
styles.add(ParagraphStyle(name='Heading1_ch', 
            parent=styles['Normal'],
            fontName='simhei',
            fontSize=16,
            leading=20,
            spaceBefore=15,
            spaceAfter=6))
styles.add(ParagraphStyle(name='Heading2_ch', 
            parent=styles['Normal'],
            fontName='simhei',
            fontSize=12,
            leading=20,
            spaceBefore=15,
            spaceAfter=6,
            alignment=TA_CENTER))
styles.add(ParagraphStyle(name='Normal_ch', 
            parent=styles['Normal'],
            fontName='simsun'))
styles.add(ParagraphStyle(name='tizhu_ch', 
            parent=styles['Normal'],
            fontName='simkai',
            alignment=TA_CENTER))
styles.add(ParagraphStyle(name='HeadTitle', 
            parent=styles['Title'],
            fontName='simhei',
            fontSize=30,
            leading=20,
            spaceBefore=20,
            spaceAfter=20))
styles.add(ParagraphStyle(name='HeadTitle_simple', 
            parent=styles['Title'],
            fontName='simhei',
            fontSize=10,
            leading=0,
            spaceBefore=0,
            spaceAfter=0))
styles.add(ParagraphStyle(name='HeadTitle_simple_left', 
            parent=styles['Title'],
            fontName='simkai',
            fontSize=10,
            leading=0,
            spaceBefore=0,
            alignment=TA_LEFT,
            spaceAfter=0))
styles.add(ParagraphStyle(name='Normal2_ch', 
            parent=styles['Normal'],
            fontName='simsun',
            fontSize=12,
            leading=20,
            spaceBefore=20))
styles.add(ParagraphStyle(name='Normalcell_ch', 
            parent=styles['Normal'],
            fontName='simsun',
            alignment=1,
            leading=20,
            spaceBefore=0,            
            spaceAfter=0))
styles.add(ParagraphStyle(name='Normalcell_ch_underline', 
            parent=styles['Normal'],
            fontName='simsun',
            alignment=1,
            leading=20,
            spaceBefore=0, 
            underlineColor=colors.black, 
            underlineWidth=0.25,          
            spaceAfter=0))
styles.add(ParagraphStyle(name='Normalcell_ch_simple', 
            parent=styles['Normal'],
            fontName='simsun',
            fontSize=8,
            alignment=1,
            leading=8,
            spaceBefore=0,            
            spaceAfter=0))
styles.add(ParagraphStyle(name='Normal_ch_center', 
            parent=styles['Normal'],
            fontName='simsun',
            fontSize=10,
            leading=0,
            spaceBefore=0,
            spaceAfter=0,
            alignment=TA_CENTER))

def gen_report_func(is_new_template):
    arm_id = get_arm_id(is_new_template)
    # result_path = "\\result" + arm_id
    # 调用模板，创建指定名称的PDF文档
    result_file_path = arm_id+".pdf" #"result\\"+arm_id+"\\result_"+"NO"+
    doc = SimpleDocTemplate(result_file_path)

    # 初始化内容
    story =[]
    # 将段落添加到内容中
    story.append(Paragraph("北京术锐技术有限公司", styles["HeadTitle_simple_left"]))
    story.append(Spacer(20, 40))
    story.append(Paragraph("手术工具性能测试记录", styles["HeadTitle_simple"]))
    # story.append(Spacer(20, 40))
    # story.append(Paragraph(['工    序：', '300 ', '编    号：', 'B'], styles["Normal_ch_center"]))
    
    story.append(Spacer(10, 12))
    cover_tbl = get_cover_tbl()
    story.append(cover_tbl)

    story.append(Spacer(20, 20))
    stereoCalib_result = get_kine_calib_result(arm_id)
    story.append(Paragraph("表A 标定结果", styles['tizhu_ch']))
    story.append(stereoCalib_result)

    story.append(Spacer(20, 20))
    fat_result_tbl, final_ok = get_fat_result(arm_id)
    story.append(Paragraph("表B 测试结果", styles['tizhu_ch']))
    t1=get_comment_sense_test(1)
    story.append(t1)
    t2=get_comment_sense_test(2)
    story.append(t2)
    story.append(fat_result_tbl)

    story.append(Spacer(20, 20))
    if final_ok:
        final_result = [['最终结论:', '合格', '', '']]
    else:
        final_result = [['最终结论:', '不合格', '', '']]
    t = Table(final_result, colWidths=[100, 100, 100, 100])
    # t = Table(cover_tbl, colWidths=[200])
    t.setStyle(TableStyle(
        [
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('SPAN',(1,0),(3,0)),
        ('FONT', (0,0), (-1,-1), 'simhei')]
        )) 
    story.append(t)
    # # 将内容输出到PDF中
    if is_new_template:
        doc.multiBuild(story, canvasmaker=FooterCanvas)
    else:
        doc.multiBuild(story, canvasmaker=FooterCanvas_old)

    print("已生成报告：\n"+result_file_path)


if __name__ == "__main__":
    import argparse
    description = "you should add those parameter" 
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("-b", "--bool", action="store_true")
    args = parser.parse_args()  
    is_new_template = args.bool
    print(is_new_template)
    gen_report_func(is_new_template)
