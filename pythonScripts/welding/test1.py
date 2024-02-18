#encoding:utf-8
import cv2
import numpy as np


ori0 = cv2.imread(r"test1.jpg")
ori = ori0[850:1050,1100:1500]
k = np.ones((7, 7), np.uint8)

# image = np.power(ori, 1)	
# ori1 = cv2.cvtColor(ori,cv2.COLOR_BGR2GRAY)
# opening = cv2.morphologyEx(ori1, cv2.MORPH_OPEN,k)       # 开运算
# opening[opening > 100] = 100
# opening[opening < 50] = 0
# # opening = np.where((opening <= 100), 255, 0)

# print(opening)

# opening = opening*2

# # cv::Rect roi_rect = cv::Rect(128, 128, roi.cols, roi.rows);

# cv2.imshow("original", ori)
# cv2.imshow("opening", opening)
# cv2.imshow("power", image)
# # cv2.imwrite(r'C:\Users\Lenovo\Desktop\opening.jpg', opening)




gray0 = cv2.cvtColor(ori, cv2.COLOR_BGR2GRAY)

# 应用拉普拉斯算子
laplacian = cv2.Laplacian(gray0, cv2.CV_8U)

# 将边缘图像与原始图像进行加权叠加
sharpened = cv2.addWeighted(gray0, 2.2, laplacian, -1.2, 0)


gray = cv2.morphologyEx(gray0, cv2.MORPH_OPEN,k)
gray2 = gray0

gray[gray > 120] = 0
# 进行边缘检测
edges = cv2.Canny(gray, 100, 200,7)

# 执行轮廓检测
contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

# 遍历检测到的轮廓
for contour in contours:
    # 进行多边形逼近
    approx = cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True)
    
    # 如果逼近的多边形具有三个顶点，则视为三角形
    # if len(approx) == 3:
    area = cv2.contourArea(contour)
    if(area>100):
        # cv2.drawContours(ori, [approx], 0, (0, 255, 0), 2)
        max_contour = approx

# 创建一个全黑的掩膜图像，与输入图像大小相同
mask = np.zeros_like(gray)

# 在掩膜上绘制最大轮廓
cv2.drawContours(mask, [max_contour], 0, 255, -1)

# 将图像与掩膜进行按位与操作，将轮廓外部的像素置为0
result = cv2.bitwise_and(gray, gray, mask=mask)

result2 = cv2.bitwise_and(result, sharpened, mask=mask)

laplacian = cv2.Laplacian(result, cv2.CV_8U)
blended = cv2.addWeighted(result, 1.5, laplacian, -0.5, 0)

enhanced = cv2.equalizeHist(result)

enhanceded = cv2.equalizeHist(enhanced)

result3 = cv2.bitwise_and(enhanced, enhanced, mask=mask)

result4 = cv2.bitwise_and(enhanceded, enhanced, mask=mask)

# 创建CLAHE对象
clahe = cv2.createCLAHE(clipLimit=100.0, tileGridSize=(8, 8))

# 应用自适应直方图均衡化
clahe_image = clahe.apply(result3)

clahe_image = cv2.equalizeHist(clahe_image)

result5 = cv2.bitwise_and(clahe_image, clahe_image, mask=mask)


# 显示识别结果
cv2.imshow('ori', ori)
cv2.imshow('gray', gray)
cv2.imshow('result', result)
cv2.imshow('result2', result2)
cv2.imshow('result4', result4)
cv2.imshow('result5', result5)
cv2.imshow('result3', result3)
cv2.imshow('gray0', gray0)
cv2.imshow('enhanced', enhanced)
cv2.imshow('enhanceded', enhanceded)
cv2.imshow('blended', blended)
cv2.waitKey()