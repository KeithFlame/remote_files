# -*- coding:utf-8 -*-
import cv2
import numpy as np
# import calcNewPixel as cnp
# import to_wx as tw
import os

def findFeaturePoints(im_left_,im_right_):
    im_left = cv2.cvtColor(im_left_,cv2.COLOR_BGR2GRAY)
    im_right = cv2.cvtColor(im_right_,cv2.COLOR_BGR2GRAY)
    # 设置SimpleBlobDetector_Params参数
    params = cv2.SimpleBlobDetector_Params()
    # 改变阈值
    params.minThreshold = 10
    params.maxThreshold = 200
    # 通过面积滤波
    params.filterByArea = True
    params.minArea = 100
    # 通过圆度滤波
    params.filterByCircularity = True
    params.minCircularity = 0.1
    # 通过凸度滤波
    params.filterByConvexity = True
    params.minConvexity = 0.87
    # 通过惯性比滤波
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    # 创建一个检测器并使用默认参数
    detector = cv2.SimpleBlobDetector_create(params)
    # 检测blobs.
    key_points_left = detector.detect(im_left)
    key_points_right = detector.detect(im_right)
    
    # 绘制blob的红点
    draw_image_left = cv2.drawKeypoints(im_left_, key_points_left, np.array([]), (0, 0, 255),
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    draw_image_right = cv2.drawKeypoints(im_right_, key_points_right, np.array([]), (0, 0, 255),
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) 
    
    return key_points_left, draw_image_left, key_points_right, draw_image_right

def getPixelPose(path_l,path_r, edge):
    # Read image
    # path_l= "./r_left/M100.jpg"
    # path_r= "./r_right/M100.jpg"
    # edge=[100, 950,1500, 1800]
    # edge=[100,576, 250, 100]
    im_l = cv2.imread(path_l)
    im_r = cv2.imread(path_r)
    im_left_ = im_l[edge[0]:edge[1],edge[2]:edge[3]]
    im_right_ = im_r[edge[0]:edge[1],edge[2]:edge[3]]
    # cv2.imshow("",im_left_)
    # cv2.imshow("",im_right_)
    # key=cv2.waitKey()
    key_points_left, draw_image_left, key_points_right, draw_image_right=findFeaturePoints(im_left_,im_right_)
    # Show blobs
    result = np.vstack((draw_image_left, draw_image_right))
    cv2.imshow("key_points", result)
    key=cv2.waitKey(200)
    # cv2.destroyAllWindows() 

    # for ele in key_points_left:
    #     print("key_points_left: ",ele.pt)
    # for ele in key_points_right:
    #     print("key_points_right: ",ele.pt)

    len1 = len(key_points_left)
    len2 = len(key_points_right)
    if len1==1 and len2==1:
        print (key_points_left[0].pt[0]+edge[2],key_points_left[0].pt[1]+edge[0], key_points_right[0].pt[0]+edge[2],key_points_right[0].pt[1]+edge[0])
    elif len1<5 and len2<5 and len1>0 and len2>0:
        for ele in key_points_left:
            print("key_points_left: ",ele.pt[0]+edge[2],ele.pt[1]+edge[0])
        for ele in key_points_right:
            print("key_points_right: ",ele.pt[0]+edge[2],ele.pt[1]+edge[0])
        print("\n")
    else:
        print("No Data")

if __name__ == '__main__':
    edge = [500, 1000,700, 1200]
    img_path_l = "./left/"
    img_path_r = "./right/"
    for i in range(28, 196):
        # name = ""
        # str_num = str(i).zfill(3)
        # name = "M"+str(i)+".jpg"
        if i<10:
            name = "M000"+str(i)+".jpg"
        elif i<100:
            name = "M00"+str(i)+".jpg"
        else:
            name = "M0"+str(i)+".jpg"
        # print(" ",name)
        # image_l = cv2.cvtColor(cv2.imread(img_path_l + name), cv2.COLOR_BGR2RGB)
        # image_r = cv2.cvtColor(cv2.imread(img_path_r + name), cv2.COLOR_BGR2RGB)
        path_l = img_path_l + name
        path_r = img_path_r + name
        # if i<111:
        # #     getPixelPose(path_l, path_r, edge1)
        #     pass
        # elif i<202:
        #     pass
        #     # getPixelPose(path_l, path_r, edge2)
        # elif i<285:
        #     # getPixelPose(path_l, path_r, edge3)
        #     pass
        # else:
        getPixelPose(path_l, path_r, edge)