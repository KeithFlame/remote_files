import cv2
import numpy as np
 
cap1 = cv2.VideoCapture(0)  # 调用目录下的视频
# cap2 = cv2.VideoCapture(1)  #调用摄像头‘0’一般是打开电脑自带摄像头，‘1’是打开外部摄像头（只有一个摄像头的情况）
 
if False == cap1.isOpened():
    print(10)
else:
    print(11)
 
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 5472)  # 设置图像宽度
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 3648)  # 设置图像高度
cap1.set(cv2.CAP_PROP_FPS , 19)   # 设置帧率
# 显示图像
iter = 1
while True:
    ret1, frame1 = cap1.read()
    # print(ret)  #
    ########图像不处理的情况
    # cv2.medianBlur(frame1,9,frame1)
    frame_1 = cv2.resize(frame1, (1350 , 900))
    cv2.imshow("frame", frame_1)
 
    input = cv2.waitKey(1)
    if input == ord('q'):
        break
    if input == ord('w'):
        str = './pic/%04d.png' % (iter)
        rc = cv2.imwrite(str, frame1)
        if rc :
            print("--> "+str+" has been saved")
        iter += 1
   
 
cap1.release()  # 释放摄像头
cv2.destroyAllWindows()  # 销毁窗口