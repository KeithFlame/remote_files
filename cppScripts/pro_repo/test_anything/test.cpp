#include <iostream>
#include <opencv.hpp>
#include <fstream>
#include <string>
#include "Jacobian_Broyden.h"

Eigen::VectorXf recombineT(Eigen::VectorXf T1, Eigen::VectorXf T2);
Eigen::Vector3f quat2eul(Eigen::Vector4f);
Eigen::MatrixXf getCurrentPose(std::string str = "./T.log");
std::vector<std::string> split(const std::string&, const std::string&);
Eigen::VectorXf getTarget(Eigen::VectorXf);
void forwardKinematics(Eigen::VectorXf psi, Eigen::Matrix4f& mat);
Eigen::VectorXf getCurrentTarget(Eigen::Matrix4f mat);
int test_main()
{
	Jacobian_Broyden jb;
	Eigen::VectorXf tar(6, 1), last_psi(6,1), cur_psi(6, 1), cur_pose(7,1), dpsi(6,1),cur_T(6,1), last_T(6, 1);
	Eigen::Matrix4f mat;
    tar << 0.f, 0.f, 0.f, 0.f, 0.f, 0.f;
    last_psi << -2.3849f,   74.9039f,    0.5608f, - 1.0987f,    2.0867f,    2.7730f;
	dpsi << 0.f, 1.f, 0.f, 0.f, 0.f, 0.f;
    jb.getTarget(tar);
	jb.getInitPsi(last_psi);
    
    forwardKinematics(last_psi,  mat);
    last_T = getCurrentTarget(mat);
    last_T << 9.19646f,  28.24f,  78.1616f, 1.6553f, - 1.2602f,    0.3259f;
    cur_pose = getTarget(last_T);
    std::cout << "cur_pose: " << cur_pose.transpose() << std::endl;
    jb.getInitPose(cur_pose);
    cur_psi = last_psi - dpsi;
    int iter = 0;


    /*
    */
    Eigen::VectorXf T(6, 1);
    T << 0.f, 0.f, 0.f, 0.f, 0.7854f, -1.5708f;
    Eigen::VectorXf TT = getTarget(T);
    std::cout << "TT.transpose(): "<<TT.transpose() << std::endl;
    





    Eigen::MatrixXf pose_lib = getCurrentPose();
    float iter_num[300];
    ////////////////////for (size_t iw = 0; iw < pose_lib.size() / 3; iw++)
    ////////////////////{
    ////////////////////    tar(0) = pose_lib(iw, 0);
    ////////////////////    tar(1) = pose_lib(iw, 1);
    ////////////////////    tar(2) = pose_lib(iw, 2);
    ////////////////////    jb.getTarget(tar);
    ////////////////////    std::cout << std::endl << "->tar: " << tar.transpose() << std::endl;
    ////////////////////    iter = 0;
    ////////////////////    while (!(jb.is_convergent()))
    ////////////////////    {
    ////////////////////        ++iter; 
    ////////////////////        forwardKinematics(cur_psi, mat);
    ////////////////////        cur_T = getCurrentTarget(mat);
    ////////////////////        cur_pose = getTarget(cur_T);
    ////////////////////        cur_psi = jb.calcNextPsi(cur_psi, cur_pose);
    ////////////////////        
    ////////////////////        jb.error;
    ////////////////////        //std::cout << "last_T: " << last_T.transpose() << std::endl;
    ////////////////////        //std::cout << "cur_T: " << cur_T.transpose() << std::endl;        
    ////////////////////        //std::cout << std::endl << "last_psi: " << last_psi.transpose() << std::endl;
    ////////////////////        //std::cout << "cur_psi: " << cur_psi.transpose() << std::endl;
    ////////////////////        ////std::cout << std::endl << "dT: " << dT.transpose() << std::endl;
    ////////////////////        //std::cout  << "dpsi: " << (cur_psi- last_psi).transpose() << std::endl;
    ////////////////////        ////std::cout << std::endl <<"Jacobian_Broyden: " << jb.JB << std::endl;
    ////////////////////        //std::cout << std::endl << "->Iteration times: " << iter << ".    Error is: " << jb.error << std::endl;
    ////////////////////        last_T = cur_T;
    ////////////////////        last_psi = cur_psi;
    ////////////////////    }
    ////////////////////    std::cout << std::endl << "->Iteration times: " << iter << ".    Error is: " << jb.error << std::endl;
    ////////////////////    iter_num[iw] = iter;
    ////////////////////}
    ////////////////////for (auto iw : iter_num)
    ////////////////////    std::cout << iw << std::endl;
    ////////////////////return 0;
	while (!(jb.is_convergent()))
	{
        //std::cout << std::endl << "->Iteration times: " << ++iter << ".    Error is: " << jb.error << std::endl;
        forwardKinematics(cur_psi, mat);
        cur_T = getCurrentTarget(mat);
        cur_pose = getTarget(cur_T);
        //Eigen::VectorXf dT = recombineT(cur_pose, getTarget(last_T));
        cur_psi = jb.calcNextPsi(cur_psi, cur_pose);
  //      std::cout << "last_T: " << last_T.transpose() << std::endl;
        std::cout << "cur_pose: " << cur_pose.transpose() << std::endl;
        std::cout << std::endl << "last_psi: " << last_psi.transpose() << std::endl;
        std::cout << "cur_psi: " << cur_psi.transpose() << std::endl;
  //      std::cout << std::endl << "dT: " << dT.transpose() << std::endl;
        std::cout  << "dpsi: " << (cur_psi- last_psi).transpose() << std::endl;
		//std::cout << std::endl <<"Jacobian_Broyden: " << jb.JB << std::endl;
        
        last_T = cur_T;
        last_psi = cur_psi;
	}
    std::cout << std::endl << "->Iteration times: " << iter << ".    Error is: " << jb.error << std::endl;
    return 0;

}

Eigen::VectorXf recombineT(Eigen::VectorXf T1, Eigen::VectorXf T2)
{
    Eigen::Vector3f vec1 = T1.head(3);
    Eigen::Vector3f vec2 = T2.head(3);
    Eigen::Vector3f vec0 = vec1 - vec2;
    if (1)
    {
        Eigen::Quaternionf quat1(T1(3), T1(4), T1(5), T1(6)), quat2(T2(3), T2(4), T2(5), T2(6));
        quat1.normalize();
        quat2.normalize();
        Eigen::AngleAxisf axang(quat2.toRotationMatrix().transpose() * quat1.toRotationMatrix());
        Eigen::VectorXf res(6, 1);
        res << vec0, axang.axis()* axang.angle() * 180.f / 3.141592654f;
        return res;
    }
    else
    {
        return vec0;
    }

}

Eigen::VectorXf getTarget(Eigen::VectorXf T)
{
	Eigen::Vector3f ea(T(3), T(4), T(5));
	Eigen::AngleAxisf ra(Eigen::AngleAxisf(ea(2), Eigen::Vector3f::UnitX()));
	Eigen::AngleAxisf pa(Eigen::AngleAxisf(ea(1), Eigen::Vector3f::UnitY()));
	Eigen::AngleAxisf ya(Eigen::AngleAxisf(ea(0), Eigen::Vector3f::UnitZ()));
    
	Eigen::Quaternionf quat = ya * pa * ra;
    std::cout << "quat.matrix: " << quat.toRotationMatrix() << std::endl;
	Eigen::VectorXf Target(7,1);
	Target << T.head(3), quat.w(), quat.x(), quat.y(), quat.z();
	return Target;
}

Eigen::MatrixXf getCurrentPose(std::string str)
{
    Eigen::MatrixXf res(300, 3);
	std::fstream fread;
	fread.open(str);
	if (!fread.is_open())
		std::cout << "ERROR::FILE_OPEN_FAILED" << std::endl;
    char buf[200];
    int ir = 0;
    
    std::vector<std::string> result;
    int z = 0;

    while (fread.getline(buf, sizeof(buf)))
    {
        result = split(buf, " ,\t");
        Eigen::VectorXf ans(6, 1);
        for (size_t i = 0; i < result.size(); i++)
            res(z,i) = atof(result[i].c_str()) + 5.f;
        z++;
    }
    return res;

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

Eigen::Vector3f quat2eul(Eigen::Vector4f vec)
{
    Eigen::Vector3f angle;
    Eigen::Quaternionf q(vec(0), vec(1), vec(2), vec(3));
    q.normalize();
    float sr_cp = 2 * (q.w() * q.x() + q.y() * q.z());
    float cr_cp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angle(2) = std::atan2f(sr_cp, cr_cp);

    float sp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sp) >= 1)
        angle(1) = std::copysignf(3.141592654f / 2, sp);
    else
        angle(1) = std::asinf(sp);

    float sy_cp = 2 * (q.w() * q.z() + q.y() * q.x());
    float cy_cp = 1 - 2 * (q.z() * q.z() + q.y() * q.y());
    angle(0) = std::atan2f(sy_cp, cy_cp);

    return angle * 180.f / 3.141592654f;
}

void forwardKinematics(Eigen::VectorXf psi, Eigen::Matrix4f& mat)
{
    float L1 = 100.f;
    float Lr = 10.f;
    float L2 = 20.f;
    float Lg = 15.f;
    float PHI = psi(0);
    float l = psi(1);
    float theta1 = psi(2);
    float delta1 = psi(3);
    float theta2 = psi(4);
    float delta2 = psi(5);
    float Ls;
    if (l < L1)
    {
        L1 = l;
        Ls = 0;
    }
    else
        Ls = l - L1;
    float k1 = theta1 / L1;
    float k2 = theta2 / L2;

    Eigen::Matrix4f T1, T2, T3, T4, T5;
    T1 << std::cosf(PHI), - std::sin(PHI), 0.f, 0.f,
        std::sinf(PHI), std::cosf(PHI), 0.f, 0.f,
        0.f, 0.f, 1.f, Ls,
        0.f, 0.f, 0.f, 1.f;
    
    if (theta1 == 0)
    {
        T2 = Eigen::Matrix4f::Identity(4, 4);
        T2(2, 3) = L1;
    }
    else
    {
        float cosTHETA1 = std::cosf(theta1);
        float sinTHETA1 = std::sinf(theta1);
        float cosDELTA1 = std::cosf(delta1);
        float sinDELTA1 = std::sinf(delta1);
        T2 << std::powf((cosDELTA1), 2) * (cosTHETA1 - 1) + 1.f, sinDELTA1* cosDELTA1* (cosTHETA1 - 1.f), cosDELTA1* sinTHETA1, cosDELTA1* (1.f - cosTHETA1) / k1,
            sinDELTA1* cosDELTA1* (cosTHETA1 - 1.f), std::powf((cosDELTA1), 2)* (1.f - cosTHETA1) + cosTHETA1, sinDELTA1* sinTHETA1, sinDELTA1* (1.f - cosTHETA1) / k1,
            -cosDELTA1 * sinTHETA1, -sinDELTA1 * sinTHETA1, cosTHETA1, sinTHETA1 / k1,
            0.f, 0.f, 0.f, 1.f;
    }
    T3 = Eigen::Matrix4f::Identity(4, 4);
    T3(2, 3) = Lr;
    if (theta2 == 0)
    {
        T4 = Eigen::Matrix4f::Identity(4, 4);
        T4(2, 3) = L2;
    }
    else
    {
        float cosTHETA1 = std::cosf(theta2);
        float sinTHETA1 = std::sinf(theta2);
        float cosDELTA1 = std::cosf(delta2);
        float sinDELTA1 = std::sinf(delta2);
        T4 << std::powf((cosDELTA1), 2) * (cosTHETA1 - 1) + 1.f, sinDELTA1* cosDELTA1* (cosTHETA1 - 1.f), cosDELTA1* sinTHETA1, cosDELTA1* (1.f - cosTHETA1) / k2,
            sinDELTA1* cosDELTA1* (cosTHETA1 - 1.f), std::powf((cosDELTA1), 2)* (1.f - cosTHETA1) + cosTHETA1, sinDELTA1* sinTHETA1, sinDELTA1* (1.f - cosTHETA1) / k2,
            -cosDELTA1 * sinTHETA1, -sinDELTA1 * sinTHETA1, cosTHETA1, sinTHETA1 / k2,
            0.f, 0.f, 0.f, 1.f;
    }
    T5 = Eigen::Matrix4f::Identity(4, 4);
    T5(2, 3) = Lg;
    mat = T1 * T2 * T3 * T4 * T5;

}

Eigen::VectorXf getCurrentTarget(Eigen::Matrix4f mat)
{
    Eigen::Matrix3f rot = mat.block(0, 0, 3, 3);
    Eigen::VectorXf vec(6, 1);
    vec << mat.block(0, 3, 3, 1), rot.eulerAngles(2, 1, 0);
    return vec;
}
