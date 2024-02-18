import cv2
import numpy as np
 
cap1 = cv2.VideoCapture(0)  # 调用目录下的视频
cap2 = cv2.VideoCapture(1)  #调用摄像头‘0’一般是打开电脑自带摄像头，‘1’是打开外部摄像头（只有一个摄像头的情况）
 
if False == cap1.isOpened():
    print(10)
else:
    print(11)
if False == cap2.isOpened():
    print(20)
else:
    print(21)
 
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 5472)  # 设置图像宽度
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 3648)  # 设置图像高度
cap1.set(cv2.CAP_PROP_FPS , 19)   # 设置帧率
cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 5472)  # 设置图像宽度
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 3648)  # 设置图像高度
cap2.set(cv2.CAP_PROP_FPS , 19)   # 设置帧率
# 显示图像
while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    # print(ret)  #
    ########图像不处理的情况
    frame_1 = cv2.resize(frame1, (1350 , 900))
    frame_2 = cv2.resize(frame2, (1350 , 900))
    img = np.concatenate((frame_1, frame_2), axis = 0) # 纵向拼接
    cv2.imshow("frame", img)
 
    input = cv2.waitKey(1)
    if input == ord('q'):
        break
    
 
cap1.release()  # 释放摄像头
cap2.release()  # 释放摄像头
cv2.destroyAllWindows()  # 销毁窗口