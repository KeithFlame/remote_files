import cv2 as cv
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
import linecache

scale = 1

K_left_new = np.mat([
    # [1.10391821e+03*scale, 0, 1.00871394e+03*scale, 0],
    # [0, 1.11844299e+03*scale, 5.46786487e+02*scale, 0],
    # [0, 0, 1, 0]
    [1096.6, 0, 1004.6, 0],
    [0, 1101.2, 547.8316,0],
    [0, 0, 1,0]
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

def draw_cor(image, trans, offset=0, K=K_left_new):
    # if trans[][1] <= 0:
    #     return image
    len1 = len(trans)
    # print(len1)
    if len1>1 :
        for i in range(0, len1-1):
            cv.line(image, (int(trans[i][0]),int(trans[i][1])), (int(trans[i+1][0]),int(trans[i+1][1])), (255, 0, 0), 4)
    # print(trans[len1-1][:])
    cv.circle(image, (int(trans[len1-1][0]),int(trans[len1-1][1])), 15, (0, 255, 255), -1)

    cv.line(image, (1565,753), (1565,145), (0, 0, 255), 8)
    cv.line(image, (1565,145), (745,145), (0, 0, 255), 8)
    cv.line(image, (745,145), (745,753), (0, 0, 255), 8)
    cv.line(image, (745,753),(1565,753), (0, 0, 255), 8)



    return image

def get_line(index, raw_data):
    line = linecache.getline(raw_data, index)
    data = line.split()
    return data

def get_trans(index, raw_data):
    data = get_line(index, raw_data)
    trans = [round(float(data[0])), round(float(data[1]))]
    return trans

def read_log(file_name, num=395):
    t = np.zeros([num, 2])

    for i in range(num):
        t[i][:] = get_trans(i+1, file_name)
        
    return t

def draw_pred_cor(img_path, output_path, pose_est_file):
    files = os.listdir(img_path)
    t = read_log(pose_est_file)
    print("",t)
    # print("q:\n",q)
    # print("t:\n",t)
    iter=1
    for filename in files:
        if not filename.endswith('.jpg'):
            continue
        first_frame_id = 1
        index = int(filename[1:4])
        image = cv.cvtColor(cv.imread(img_path + filename), cv.COLOR_BGR2RGB)
        print('Read img', img_path + filename)
        # print("12321312",t[0:iter][:])
        img_with_pose = draw_cor(image, t[0:iter][:], K=K_left_new)
        cv.imwrite(output_path + filename, cv.cvtColor(img_with_pose, cv.COLOR_RGB2BGR))
        print('Write img', output_path + filename)
        iter+=1

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
    img_path = "./r_right/"
    output_path = "./after_pic/"
    pred_data = "right_pix.log"

    draw_pred_cor(img_path, output_path, pred_data)