#include "config.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>



Eigen::Matrix3d left_camera_intrinsic;
Eigen::Vector4d left_camera_distortion;
Eigen::Matrix3d right_camera_intrinsic;
Eigen::Vector4d right_camera_distortion;

Eigen::Matrix3d rect_left_camera;
Eigen::Matrix3d rect_right_camera;
Eigen::Matrix3d A_cam;
double b_dis;

bool loadConfig()
{
    Eigen::Matrix3d rotation_right_left_T;
    Eigen::Vector3d translation_right_left;

    //limit
    //left_camera_intrinsic << 2611.22955626447, 0, 1268.57244632280,
    //    0, 2613.10154350395, 678.966658211952,
    //    0, 0, 1;

    //left_camera_distortion << -0.148141649397529, 0.175489017937266, -0.000357934603870964, -0.000268862412337595;

    //right_camera_intrinsic << 2573.51226734968, 0, 1193.68744398003,
    //    0, 2572.93982339836, 657.235501957461,
    //    0, 0, 1;

    //right_camera_distortion << -0.132584591441184, 0.119831793227610, -0.000622755620871696, -0.00165274380634398;

    //rotation_right_left_T << 0.999998819847400, -0.000431638971866026, -0.00147444620287730,
    //    0.000425868719151440, 0.999992259059611, -0.00391157982020148,
    //    0.00147612317956909, 0.00391094728342451, 0.999991262737683;

    //translation_right_left << -50.6237163630020, -0.329192808658008, -0.266388216519978;


    //up_new_0921-2021
    //left_camera_intrinsic << 2574.64982376944,	0,	1340.60676614705,
    //    0,	2574.29460124175,	856.974396801785,
    //    0,	0,	1;
    //left_camera_distortion << -0.132134305648913, 0.118654854686071, 0.000172316002050611, -0.000152835096902185;

    //right_camera_intrinsic << 2557.26150041756,	0,	1283.94117957334,
    //    0,	2557.07456318283,	736.492073027955,
    //    0,	0,	1;
    //right_camera_distortion << -0.135738877608740, 0.133105571042938, 2.02609689670337e-05, 0.000163017809780048;

    //rotation_right_left_T << 0.999992538520314, -0.00369124208951367, -0.00113913806707813,
    //0.00369384204969463, 0.999990558437172, 0.00228879169522447,
    //0.00113067882759496, -0.00229298241354431, 0.999996731893180;

    //translation_right_left << -50.2791175774307, - 0.509913285452749,	0.296629049553616;

    //up_new_0924-2021
    left_camera_intrinsic << 2574.15745985723,	0,	1340.57250676268,
        0,	2573.47648339799,	858.376878291841,
        0,	0,	1;
    left_camera_distortion << -0.127591898653533,	0.112105617833515, 6.73680187064503e-05, - 0.000207068552555372;

    right_camera_intrinsic << 2553.61565896593,	0,	1282.43303228252,
        0,	2552.95300696449,	737.115034711237,
        0,	0,	1;
    right_camera_distortion << -0.137093900572838,	0.136796158227094, 0.000176391412009676,	0.000195303705407776;

    rotation_right_left_T << 0.999991848026499, - 0.00369426556432345, - 0.00162981056768980,
        0.00369764999123913,	0.999991003624873,	0.00207847392630267,
        0.00162211747065023, - 0.00208448345166967,	0.999996511825742;

    translation_right_left << -50.1746811359027, - 0.450956519000691, - 0.114480650724395;

    auto rotation_right_left = rotation_right_left_T.transpose();
    Eigen::AngleAxisd r_axang;
    r_axang.fromRotationMatrix(rotation_right_left);
    Eigen::AngleAxisd r_axang_n1 = r_axang, r_axang_n2 = r_axang;
    r_axang_n1.angle() = 0.5 * r_axang.angle();
    r_axang_n2.angle() = -0.5 * r_axang.angle();
    auto rect_left_1 = r_axang_n1.toRotationMatrix();
    auto rect_right_1 = r_axang_n2.toRotationMatrix();

    Eigen::Vector3d x_before = rect_right_1 * translation_right_left;
    Eigen::Vector3d x_after = (x_before(0) > 0 ? 1 : -1) * Eigen::Vector3d(1, 0, 0);

    Eigen::AngleAxisd r_axang2;
    auto rot_axis2 = x_before.cross(x_after);

    auto rot_angle2 = acos(x_before.dot(x_after) / (x_before.norm() * x_after.norm()));
    r_axang2.axis() = rot_axis2;
    r_axang2.angle() = rot_angle2;

    auto r_align = r_axang2.toRotationMatrix();

    rect_left_camera = r_align * rect_left_1;
    rect_right_camera = r_align * rect_right_1;

    Eigen::Vector4d f_vec;
    f_vec << left_camera_intrinsic(0, 0), left_camera_intrinsic(1, 1),
        right_camera_intrinsic(0, 0), right_camera_intrinsic(1, 1);

    float f_comm = f_vec.maxCoeff() * 1.05;
    float cx_comm = 0.5 * (left_camera_intrinsic(0, 2) + right_camera_intrinsic(0, 2));
    float cy_comm = 0.5 * (left_camera_intrinsic(1, 2) + right_camera_intrinsic(1, 2));

    A_cam << f_comm, 0, cx_comm,
        0, f_comm, cy_comm,
        0, 0, 1;//offline before year
    b_dis = translation_right_left.norm();

    //A_cam << 1100.95, 0, 984.237335,
    //    0, 1100.95, 561.2881546,
    //    0, 0, 1;  //before year

    //A_cam << 1109.4, 0, 1014.62313,
    //    0, 1100.4, 560.842061,
    //    0, 0, 1;    //after year
    //b_dis = 4.0496;

        /* 20200513_8#_1*/
    //A_cam << 1084.4, 0, 1167.4227,
    //    0, 1084.4, 534.755,
    //    0, 0, 1;
    //b_dis = 4.1053;

    /* 20200513_8#_1_1*/
    //A_cam << 1079.84533, 0, 1143.35767 - 1,
    //    0, 1079.84533, 537.1847 -1 ,
    //    0, 0, 1;
    //b_dis = 4.04127;

    /* 20200513_8#_2*/
    //A_cam << 1085.619, 0, 1021.602,
    //    0, 1085.619, 530.591,
    //    0, 0, 1;
    //b_dis = 3.91603759961283;

    /* 20200513_8#_1_3*/
    //A_cam << 1083.33766, 0, 1312.909378,
    //    0, 1083.33766, 535.522778,
    //    0, 0, 1;
    //b_dis = 3.9378108800258;

    /* 20200513_8#_1_4*/
    //A_cam << 1092.5634723, 0, 1079.3544464,
    //    0, 1092.5634723, 529.98729706,
    //    0, 0, 1;
    //b_dis = 4.00340574480825;

    /* 20200524_8#_1_1*/
    //A_cam << 1095.6637, 0, 998.6495,
    //    0, 1095.6637, 528.3884,
    //    0, 0, 1;
    //b_dis = 4.26273551125839;

    /* 20200607_11#_1*/
    //A_cam << 1111.882, 0, 1128.686,
    //    0, 1111.882, 579.405,
    //    0, 0, 1;
    //b_dis = 4.1700997217286;

    return true;
}

Eigen::Matrix3d getMatrix3dParameter(std::string matName, bool isUp)
{
    Eigen::Matrix3d mat;
    std::fstream fread;
    std::string parameterPath = getParaPath();
    //std::cout << "parameterPath: " << parameterPath << std::endl;
    std::string str;
    if(isUp)
        str = "./cameraPara/"+parameterPath + "/up/" + matName + ".log";
    else
        str = "./cameraPara/"+parameterPath + "/down/" + matName + ".log";
    std::vector<std::string> result;
    std::cout << "开始读取文件..." << std::endl;
    //获取Ai
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return mat ;
    }
    char buf[200];
    int ir = 0;
    while (fread.getline(buf, sizeof(buf)))
    {
        std::cout << matName<<":   " << ir << std::endl;
        std::cout << "buf" << ":   " << buf << std::endl;
        result = split(buf, " ,\t");
        std::cout << result[0].c_str() <<"   "<< result[1].c_str()<<"   "<< result[2].c_str() << std::endl;
        mat(ir, 0) = atof(result[0].c_str());
        mat(ir, 1) = atof(result[1].c_str());
        mat(ir, 2) = atof(result[2].c_str());
        ir++;
        
    }
    std::cout << mat << std::endl;
    fread.close();
    return mat;
}

Eigen::Vector4d getVector4dParameter(std::string vecName, bool isUp)
{
    Eigen::Vector4d vec;
    std::fstream fread;
    std::string parameterPath = getParaPath();
    std::string str;
    if (isUp)
        str = "./cameraPara/" + parameterPath + "/up/" + vecName + ".log";
    else
        str = "./cameraPara/" + parameterPath + "/down/" + vecName + ".log";
    std::vector<std::string> result;
    std::cout << "开始读取文件..." << std::endl;
    //获取Ai
    fread.open(str);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        return vec;
    }
    char buf[200];
    int ir = 0;
    fread.getline(buf, sizeof(buf));
    result = split(buf, " ,\t");
    vec(0) = atof(result[0].c_str());
    vec(1) = atof(result[1].c_str());
    vec(2) = atof(result[2].c_str());
    if (result.size() > 3 && result.size() != 0)
        vec(3) = atof(result[3].c_str());
    else
        vec(3) = 0;
    std::cout << vec << std::endl;
    fread.close();
    return vec;
}

std::string getParaPath()
{
    std::fstream fread;
    std::string str =  "./cameraPara/paraPath.log";
    std::vector<std::string> result;
    std::cout << "开始读取文件..." << std::endl;
    std::cout << str << std::endl;
    //获取Ai
    fread.open(str,std::ios::in);
    if (!fread.is_open())
    {
        std::cout << "读取文件" + str + "失败" << std::endl;
        fread.close();
        return "system01";
    }
    char buf[200];
    int ir = 0;
    fread.getline(buf, sizeof(buf));
    std::string res = buf;
    fread.close();
    return res;
}

bool loadConfigV2(bool normal)
{
    Eigen::Matrix3d rotation_right_left_T;
    Eigen::Vector3d translation_right_left;
    Eigen::Vector4d tem;


    if (normal)
    {

        ////左相机内参:
        //left_camera_intrinsic << 2570.558026f, 0.000000f, 1337.346376f,
        //    0.000000f, 2569.860441f, 861.660267f,
        //    0.000000f, 0.000000f, 1.000000f;
        ////左相机畸变:
        //left_camera_distortion << -0.132964f, 0.127653f, 0.000356f, -0.000129f;
        ////右相机内参:
        //right_camera_intrinsic << 2555.085704f, 0.000000f, 1287.181711f,
        //    0.000000f, 2554.107075f, 736.374764f,
        //    0.000000f, 0.000000f, 1.000000f;
        ////右相机畸变:
        //right_camera_distortion << -0.136526f, 0.148733f, -0.000178f, 0.000579f;
        ////相机外参:
        //rotation_right_left_T << 0.999993f, -0.003557f, 0.000897f,
        //    0.003557f, 0.999993f, 0.000778f,
        //    -0.000900f, -0.000775f, 0.999999f;
        ////相机外参:
        //translation_right_left << -50.182075f, -0.475732f, 0.322263f;
        std::string fileName;
        fileName = "leftCameraIntrinsic";
        left_camera_intrinsic = getMatrix3dParameter(fileName);
        fileName = "rightCameraIntrinsic";
        right_camera_intrinsic = getMatrix3dParameter(fileName);
        fileName = "rotationRightToLeft";
        rotation_right_left_T = getMatrix3dParameter(fileName);

        fileName = "leftCameraDistortion";
        left_camera_distortion = getVector4dParameter(fileName);
        fileName = "rightCameraDistortion";
        right_camera_distortion = getVector4dParameter(fileName);

        fileName = "translationRightToLeft";
        tem = getVector4dParameter(fileName);
        translation_right_left = Eigen::Vector3d{ tem(0),tem(1),tem(2) };
    }
    else
    {
        //limit
        //left_camera_intrinsic << 2611.22955626447, 0, 1268.57244632280,
        //    0, 2613.10154350395, 678.966658211952,
        //    0, 0, 1;

        //left_camera_distortion << -0.148141649397529, 0.175489017937266, -0.000357934603870964, -0.000268862412337595;

        //right_camera_intrinsic << 2573.51226734968, 0, 1193.68744398003,
        //    0, 2572.93982339836, 657.235501957461,
        //    0, 0, 1;

        //right_camera_distortion << -0.132584591441184, 0.119831793227610, -0.000622755620871696, -0.00165274380634398;

        //rotation_right_left_T << 0.999998819847400, -0.000431638971866026, -0.00147444620287730,
        //    0.000425868719151440, 0.999992259059611, -0.00391157982020148,
        //    0.00147612317956909, 0.00391094728342451, 0.999991262737683;

        //translation_right_left << -50.6237163630020, -0.329192808658008, -0.266388216519978;
        //左相机内参:
        //left_camera_intrinsic << 2575.354738f, 0.000000f, 1194.449349f,
        //    0.000000f, 2574.044601f, 656.516836f,
        //    0.000000f, 0.000000f, 1.000000f;
        ////左相机畸变:
        //left_camera_distortion << -0.127792f, 0.116602f, -0.001539f, -0.001354f;
        ////右相机内参:
        //right_camera_intrinsic << 2573.187039f, 0.000000f, 1305.114598f,
        //    0.000000f, 2573.497046f, 973.323759f,
        //    0.000000f, 0.000000f, 1.000000f;
        ////右相机畸变:
        //right_camera_distortion << -0.123400f, 0.100395f, -0.000371f, 0.000293f;
        ////相机外参:
        //rotation_right_left_T << 0.999993f, -0.002883f, 0.002481f,
        //    0.002859f, 0.999952f, 0.009368f,
        //    -0.002508f, -0.009361f, 0.999953f;
        ////相机外参:
        //translation_right_left << -50.133390f, -0.220889f, -0.045294f;
        std::string fileName;
        fileName = "leftCameraIntrinsic";
        left_camera_intrinsic = getMatrix3dParameter(fileName,0);
        fileName = "rightCameraIntrinsic";
        right_camera_intrinsic = getMatrix3dParameter(fileName,0);
        fileName = "rotationRightToLeft";
        rotation_right_left_T = getMatrix3dParameter(fileName,0);

        fileName = "leftCameraDistortion";
        left_camera_distortion = getVector4dParameter(fileName,0);
        fileName = "rightCameraDistortion";
        right_camera_distortion = getVector4dParameter(fileName,0);

        fileName = "translationRightToLeft";
        tem = getVector4dParameter(fileName,0);
        translation_right_left = Eigen::Vector3d{ tem(0),tem(1),tem(2) };

    }
    

    auto rotation_right_left = rotation_right_left_T.transpose();
    Eigen::AngleAxisd r_axang;
    r_axang.fromRotationMatrix(rotation_right_left);
    Eigen::AngleAxisd r_axang_n1 = r_axang, r_axang_n2 = r_axang;
    r_axang_n1.angle() = 0.5 * r_axang.angle();
    r_axang_n2.angle() = -0.5 * r_axang.angle();
    auto rect_left_1 = r_axang_n1.toRotationMatrix();
    auto rect_right_1 = r_axang_n2.toRotationMatrix();

    Eigen::Vector3d x_before = rect_right_1 * translation_right_left;
    Eigen::Vector3d x_after = (x_before(0) > 0 ? 1 : -1) * Eigen::Vector3d(1, 0, 0);

    Eigen::AngleAxisd r_axang2;
    auto rot_axis2 = x_before.cross(x_after);

    auto rot_angle2 = acos(x_before.dot(x_after) / (x_before.norm() * x_after.norm()));
    r_axang2.axis() = rot_axis2;
    r_axang2.angle() = rot_angle2;

    auto r_align = r_axang2.toRotationMatrix();

    rect_left_camera = r_align * rect_left_1;
    rect_right_camera = r_align * rect_right_1;

    Eigen::Vector4d f_vec;
    f_vec << left_camera_intrinsic(0, 0), left_camera_intrinsic(1, 1),
        right_camera_intrinsic(0, 0), right_camera_intrinsic(1, 1);

    float f_comm = f_vec.maxCoeff() * 1.05;
    float cx_comm = 0.5 * (left_camera_intrinsic(0, 2) + right_camera_intrinsic(0, 2));
    float cy_comm = 0.5 * (left_camera_intrinsic(1, 2) + right_camera_intrinsic(1, 2));

    A_cam << f_comm, 0, cx_comm,
        0, f_comm, cy_comm,
        0, 0, 1;//offline before year
    b_dis = translation_right_left.norm();

    //A_cam << 1100.95, 0, 984.237335,
    //    0, 1100.95, 561.2881546,
    //    0, 0, 1;  //before year

    //A_cam << 1109.4, 0, 1014.62313,
    //    0, 1100.4, 560.842061,
    //    0, 0, 1;    //after year
    //b_dis = 4.0496;

        /* 20200513_8#_1*/
    //A_cam << 1084.4, 0, 1167.4227,
    //    0, 1084.4, 534.755,
    //    0, 0, 1;
    //b_dis = 4.1053;

    /* 20200513_8#_1_1*/
    //A_cam << 1079.84533, 0, 1143.35767 - 1,
    //    0, 1079.84533, 537.1847 -1 ,
    //    0, 0, 1;
    //b_dis = 4.04127;

    /* 20200513_8#_2*/
    //A_cam << 1085.619, 0, 1021.602,
    //    0, 1085.619, 530.591,
    //    0, 0, 1;
    //b_dis = 3.91603759961283;

    /* 20200513_8#_1_3*/
    //A_cam << 1083.33766, 0, 1312.909378,
    //    0, 1083.33766, 535.522778,
    //    0, 0, 1;
    //b_dis = 3.9378108800258;

    /* 20200513_8#_1_4*/
    //A_cam << 1092.5634723, 0, 1079.3544464,
    //    0, 1092.5634723, 529.98729706,
    //    0, 0, 1;
    //b_dis = 4.00340574480825;

    /* 20200524_8#_1_1*/
    //A_cam << 1095.6637, 0, 998.6495,
    //    0, 1095.6637, 528.3884,
    //    0, 0, 1;
    //b_dis = 4.26273551125839;

    /* 20200607_11#_1*/
    //A_cam << 1111.882, 0, 1128.686,
    //    0, 1111.882, 579.405,
    //    0, 0, 1;
    //b_dis = 4.1700997217286;

    return true;
}


std::vector<std::string> split(const std::string& s, const std::string& seperator)
{
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while (i != s.size()) {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[i] == seperator[x]) {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[j] == seperator[x]) {
                    flag = 1;
                    break;
                }
            if (flag == 0)
                ++j;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}