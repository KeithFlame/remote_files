import cv2 as cv
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import linecache
import sys


scale = 1

K_left_new = np.asmatrix([
    # [1.10391821e+03*scale, 0, 1.00871394e+03*scale, 0],
    # [0, 1.11844299e+03*scale, 5.46786487e+02*scale, 0],
    # [0, 0, 1, 0]
    # [1096.6, 0, 1004.6, 0],
    # [0, 1101.2, 547.8316,0],
    # [0, 0, 1,0]
    [1089.2,	0,	965.353,0],
        [0,	1089.18,	522.076,0],
        [0,	0,	1,0]
])

def R2quat(mat):
    quat = R.from_matrix(mat).as_quat()
    return quat

def quat2R(quat):
    assert len(quat) == 4
    mat = R.from_quat(quat).as_matrix()
    return mat

def space_cor_to_image_cor(point, K=K_left_new):
    point = np.row_stack((point, [1]))

    image_point = K * point / point[2]
    return image_point[0], image_point[1]

def draw_line(image, point, color, point_new, offset=160, K=K_left_new):
    u, v = space_cor_to_image_cor(point, K)
    u_new, v_new = space_cor_to_image_cor(point_new, K)
    cv.line(image, (int(u  - offset * scale), int(v)), (int(u_new - offset * scale), int(v_new)), color, 2)
    return image

# def draw_point(image, uv, is_left=True):
#    
#     u = uv[0]
#    v = uv[1]
#
#    '''Draw the base point'''
#    if is_left:
#        cv.circle(image, (int(u - 160 * scale), int(v)),
#                  3, (255, 255, 255), -1)
#    else:
#        cv.circle(image, (int(u), int(v)), 3, (255, 255, 255), -1)
#
#    return image

def draw_cor(image, trans, color, offset=0, K=K_left_new):
    # if trans[2] <= 0:
    #     return image
    len1=len(trans)
    t = np.zeros([len1, 2])
    for i in range(len1):
        point = np.asmatrix([
                [trans[i][0]],
                [trans[i][1]],
                [trans[i][2]]
            ])
        u, v = space_cor_to_image_cor(point, K)
        t[i][0]=u
        t[i][1]=v
    
    clo = (0,0,0)
    wid = 4
    if color == 1:
        clo = (0,0,255)
        wid = 8
        '''Draw the base point'''
        # # if len1>1 :
        # # print("",len1)
        # # print("",t)
        # if len1>0 :
        #     cv.circle(image, (int(t[len1-1][0]),int(t[len1-1][1])), 5, clo, 3)
    elif color == 2:
        clo = (200,200,200)
        wid = 8
    else:
        clo = (255,0,0)
        wid = 4
        
    # print(t)
    # if len1>1 :
    #     for i in range(0, len1-1):
    #         cv.line(image, (int(t[i][0]),int(t[i][1])), (int(t[i+1][0]),int(t[i+1][1])), clo, wid)
    
    intersect = 392 #这个数字需要根据情况自助修改
    if len1 < intersect:    
        if len1>1:
            for i in range(0, len1-1):
                cv.line(image, (int(t[i][0]),int(t[i][1])), (int(t[i+1][0]),int(t[i+1][1])), (255, 0, 0), 4)
            
        # cv.circle(image, (int(trans[len1-1][0]),int(trans[len1-1][1])), 15, (255, 0, 0), -1)

    if len1>intersect:
        t2 = t[intersect-1:len1-1][:]
        for i in range(1, len1-intersect-1):
            cv.line(image, (int(t2[i][0]),int(t2[i][1])), (int(t2[i+1][0]),int(t2[i+1][1])), (0, 0, 255), 4)
        
        # cv.circle(image, (int(trans[len1-1][0]),int(trans[len1-1][1])), 15, (0, 0, 255), -1)



    return image

def get_line(index, raw_data):
    line = linecache.getline(raw_data, index)
    data = line.split()
    return data

def get_trans(index, raw_data):
    data = get_line(index, raw_data)
    trans = [float(data[0]), float(data[1]), float(data[2])]
    return trans

def get_quat(index, raw_data):
    data = get_line(index, raw_data)
    # print("",data)
    quat = [float(data[4]), float(data[5]), float(data[6]), float(data[3])]
    return quat

def read_log(file_name):
    count = count_nonempty_lines(file_name)
    print("count: ", count)
    # print("", file_name)
    q = np.zeros([count, 4])
    t = np.zeros([count, 3])

    for i in range(count):
        q[i][:] = get_quat(i+1, file_name)
        t[i][:] = get_trans(i+1, file_name)
        
    return q, t

def count_nonempty_lines(file_path):
    count = 0
    with open(file_path, 'r') as file:
        for line in file:
            if line.strip():  # 检查去除空白字符后的行是否为空
                count += 1
    return count
    
def draw_pred_cor(img_path, output_path, pose_est_file, target_file,cur_target_file):
    files = os.listdir(img_path)
    q, t = read_log(pose_est_file)
    qt,tt = read_log(target_file)
    qtt,ttt = read_log(cur_target_file)
    # print("qt:\n",qt)
    # print("tt:\n",tt)
    iter= 0 
    for filename in files:
        if not filename.endswith('.jpg'):
            continue
        first_frame_id = 0
        index = int(filename[1:4])
        image = cv.cvtColor(cv.imread(img_path + filename), cv.COLOR_BGR2RGB)
        print('Read img', img_path + filename)
        # img_with_pose = draw_cor(image, tt, 2, K=K_left_new)
        # img_with_pose = draw_cor(img_with_pose, ttt[0:(index - first_frame_id)][:], 1, K=K_left_new)
        img_with_pose = draw_cor(image, t[0:(index - first_frame_id)][:], 3, K=K_left_new)
        
        
        cv.imwrite(output_path + filename, cv.cvtColor(img_with_pose, cv.COLOR_RGB2BGR))
        print('Write img', output_path + filename)

        # iter += 1
        # if iter == 2:
        #     break

def plotAndShow(vec,pic_left):
    t=vec[0:3]
    q=vec[3:7]
    image = cv.cvtColor(pic_left, cv.COLOR_BGR2RGB)
    img_with_pose = draw_cor(image, t, q, K=K_left_new)
    img_with_pose=cv.cvtColor(img_with_pose, cv.COLOR_RGB2BGR)
    # cv.imshow("img_with_pose",img_with_pose)
    # key=cv.waitKey()
    return img_with_pose


if __name__ == '__main__':

    args = sys.argv[1:]  # 忽略脚本文件名，只获取参数
    # 在命令行打印参数
    for arg in args:
        print(arg)
    
    arg = args[0]
    img_path = arg+"/after_pic/"
    output_path = arg+"/final_pic/"
    current_data = arg+"/cur_data_3.log"
    target_data = arg+"/tar_data.log"
    cur_target_data = arg+"/cur_tar_data.log"
    
    draw_pred_cor(img_path, output_path, current_data, target_data, cur_target_data)