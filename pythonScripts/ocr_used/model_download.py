import os
from urllib.request import urlretrieve


def download_tessdata(url, savepath='./'):
    # 显示下载进度
    def reporthook(a, b, c):
        print("\rdownloading: %5.1f%%" % (a * b * 100.0 / c), end="")

    filename = os.path.basename(url)
    if not os.path.isfile(os.path.join(savepath, filename)):
        print('Downloading data from %s' % url)
        urlretrieve(url, os.path.join(savepath, filename), reporthook=reporthook)
        print('\nDownload finished!')
    else:
        print('File already exsits!')

    filesize = os.path.getsize(os.path.join(savepath, filename))  # 获取文件大小
    print('File size = %.2f Mb' % (filesize / 1024 / 1024))  # Bytes转换为Mb


tessdata_dir = './tessdata/'
tessdata_url = 'https://ghproxy.com/https://raw.githubusercontent.com/tesseract-ocr/tessdata/master/{}.traineddata'

# 语言： 中+英
lang = 'chi_sim+eng'
for lang_name in lang.split('+'):
    download_tessdata(tessdata_url.format(lang_name), tessdata_dir)