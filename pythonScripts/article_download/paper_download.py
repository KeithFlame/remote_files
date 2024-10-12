# import requests
# from bs4 import BeautifulSoup

# # 设置SciHub的网址
# base_url = 'https://sci-hub.se/'

# # 读取包含DOI号码的文件
# with open('doi.log', 'r') as f:
#     doi_list = f.readlines()

# # 遍历DOI号码列表
# for doi in doi_list:
#     # 构造请求SciHub的URL
#     url = base_url + doi.strip()
#     # 发送请求
#     response = requests.get(url)
#     # 解析HTML页面
#     soup = BeautifulSoup(response.text, 'html.parser')
#     # 获取文献的下载链接
#     iframe_tag = soup.find('iframe')
#     download_link=[]
#     if iframe_tag is not None:
#         download_link = iframe_tag.get('src')
#         # 继续处理 download_link
#         download_link = soup.find('iframe')['src']
#     else:
#         print("<iframe> 标签未找到")
#         continue

    
#     # 下载文献
#     response = requests.get(download_link)
#     # 保存文献到本地
#     with open(doi.strip() + '.pdf', 'wb') as f:
#         f.write(response.content)

# -*- coding: utf-8 -*-
"""
Created on Sun Jun  6 21:09:44 2021

@author: dell
"""


"""
这是原作者的信息
@File: version_1.1_doi_to_get_pdf.py
@Time: 2021/4/20 10:10 下午
@Author: genqiang_wu@163.com
@desc: 通过doi号下载文献pdf
"""

import requests
import re
import os
import urllib.request
import openpyxl

# headers 保持与服务器的会话连接
headers = {
    'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/63.0.3239.108 Safari/537.36',
}

'''
根据doi，找到文献的pdf，然后下载到本地
'''


def getPaperPdf(url):
    pattern = '/.*?\.pdf'
    content = requests.get(url, headers=headers)
    download_url = re.findall(pattern, content.text)
    # print(download_url)
    download_url[1] = "https:" + download_url[1]
    print(download_url[1])
    path = r"papers"
    if os.path.exists(path):
        pass
    else:
        os.makedirs(path)

    # 使用 urllib.request 来包装请求
    req = urllib.request.Request(download_url[1], headers=headers)
    # 使用 urllib.request 模块中的 urlopen方法获取页面
    u = urllib.request.urlopen(req, timeout=5)

    file_name = download_url[1].split('/')[-2] + '%' + download_url[1].split('/')[-1]
    f = open(path + '/' + file_name, 'wb')

    block_sz = 8192
    while True:
        buffer = u.read(block_sz)
        if not buffer:
            break
        f.write(buffer)
    f.close()
    print("Sucessful to download" + " " + file_name)
'''
将表格放在代码保存和运行的路径内，将wb变量内的'n0606.xlsx'改为自己的excel文件名,
最后下载的论文在该路径下新建的papers文件夹内
'''
wb = openpyxl.load_workbook('paper_list.xlsx')
#doi在sheet1中
sheet1 = wb.get_sheet_by_name('Sheet1')
#读取第A列

'''
修改代码内，excel中DOI所在列，我的在BC，所以col_range变量后面的字符改为了‘BC’
'''
col_range = sheet1['C']
# 读取其中的第几行：row_range = sheet1[2:6]
fails=[]


 #以下代码加入了我找的其他SCI-hub网址，不需要可以删除一些
i = 0
for col in col_range: # 打印BC两列单元格中的值内容
    i = i + 1
    if i == 4:
        break
    doi=col.value
    doi = "10.1016/j.urology.2020.05.041"
    print (doi)
    if __name__ == '__main__':
        sci_Hub_Url = "https://sci-hub.ren/"
        paper_url = sci_Hub_Url + doi
        print(paper_url)
        nmm=0
        try:
            getPaperPdf(paper_url)  # 通过文献的url下载pdf
            continue
        except Exception:
            nmm=1
            print("Failed to get pdf 1"  )         
        if nmm==1:
            try :
                sci_Hub_Url_2 = "https://sci-hub.se/"
                paper_url_2 = sci_Hub_Url_2 + doi
                getPaperPdf(paper_url_2)
                
                continue
            except Exception:
                print("Failed to get pdf 2")
        if nmm==1:
            try :
                sci_Hub_Url_3 = "https://sci-hub.st/"
                paper_url_3 = sci_Hub_Url_3 + doi
                getPaperPdf(paper_url_3)
                continue
            except Exception:
                print("Failed to get pdf 3")
        if nmm==1:
            try :
                sci_Hub_Url_4 = "https://sci-hub.shop/"
                paper_url_4 = sci_Hub_Url_4 + doi
                getPaperPdf(paper_url_4)
                continue
            except Exception:
                print("Failed to get pdf 4")
        if nmm==1:
            try :
                sci_Hub_Url_5 = "https://sci-hub.shop/"
                paper_url_5 = sci_Hub_Url_5 + doi
                getPaperPdf(paper_url_5)
                continue
            except Exception:
                print("Failed to get pdf 5")          
        if nmm==1:
            try :
                sci_Hub_Url_7 = "https://sci-hub.do/"
                paper_url_7 = sci_Hub_Url_7 + doi
                getPaperPdf(paper_url_7)
                continue
            except Exception:
                print("Failed to get pdf 7")    
        if nmm==1:
            try :
                sci_Hub_Url_6 = "https://libgen.ggfwzs.net/"
                paper_url_6 = sci_Hub_Url_6 + doi
                getPaperPdf(paper_url_6)
                continue
            except Exception:
                print("Failed to get pdf 6")
                fails.append(doi)
                
#获取下载失败的doi
print (fails)