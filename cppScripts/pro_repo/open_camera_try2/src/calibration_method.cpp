#include "calibration_method.h"
#include <Eigen/Geometry>


CalibrationMethod::CalibrationMethod()
{
}

CalibrationMethod::~CalibrationMethod()
{
}

void CalibrationMethod::setInitValue(const Eigen::VectorXd&)
{

}

const Eigen::VectorXd CalibrationMethod::getFinalValue()
{
    return Eigen::VectorXd();
}

Eigen::Vector3d CalibrationMethod::JvLCouple(double theta, double delta, double L1, double L0, double zeta)
{
    double theta0(0), theta1(0);
    if (L1 + L0 * zeta < 0.0001) {
        theta = 0;
        double theta0 = 0;
    }
    else {
        double theta1 = theta * L1 / (L1 + L0 * zeta);
        double theta0 = theta * L0 * zeta / (L1 + L0 * zeta);
    }

    Eigen::Vector3d JvL;

    if (L0 < 0.00001) {
        if (std::abs(theta1) < 1e-7) {
            JvL << 0, 0, 1;
        }
        else {
            double c = (1 - cos(theta1)) / theta1;
            double s = sin(theta1) / theta1;
            JvL << c * cos(delta), c* sin(delta), s;
        }
    }
    else {
        if (std::abs(theta1) < 1e-7) {
            JvL << 0, 0, 1;
        }
        else {
            if (std::abs(zeta) > 0.001) {
                double c1 = (-zeta * cos(theta) + (zeta - 1) * cos(theta0) - (zeta - 1) * theta1 * sin(theta0) + 1) / theta;
                double c2 = (-zeta * cos(theta) + (zeta - 1) * cos(theta0) - (zeta - 1) * theta1 * sin(theta0) + 1) / theta;
                double c3 = (zeta * sin(theta) - (zeta - 1) * sin(theta0) - (zeta - 1) * theta1 * cos(theta0)) / theta;
                JvL << c1 * cos(delta), c2* sin(delta), c3;
            }
            else {
                JvL << 0, 0, 1;
            }
        }
    }
    return JvL;
}

Eigen::MatrixXd CalibrationMethod::JvSegCouple(double theta, double delta, double L1, double L0, double zeta)
{
    Eigen::MatrixXd Jv(3, 2);

    if (L1 + L0 * zeta < 0.0001) {
        Jv.setZero();
    }
    else {
        double theta1 = theta * L1 / (L1 + L0 * zeta);
        double theta0 = theta * L0 * zeta / (L1 + L0 * zeta);

        if (zeta > 0.001) {
            if (std::abs(theta1) < 1e-7) {
                Jv << L1 / 2 * (zeta * L0 * L0 / (L1 * L1) + 2 * zeta * L0 / L1 + 1) * cos(delta) * L1 / (L1 + zeta * L0), 0,
                    L1 / 2 * (zeta * L0 * L0 / (L1 * L1) + 2 * zeta * L0 / L1 + 1) * sin(delta) * L1 / (L1 + L0 * zeta), 0,
                    0, 0;
            }
            else {
                double B1 = L1 / theta1 * (1 / zeta + cos(theta0) * (1 - 1 / zeta - cos(theta1)) + sin(theta0) * sin(theta1));
                double B2 = L1 / theta1 * (cos(theta0) * sin(theta1) - sin(theta0) * (1 - 1 / zeta - cos(theta1)));
                double C1 = L1 / theta1 * sin(theta0) * cos(theta1) - zeta * L0 / theta1 * sin(theta0) * (1 - 1 / zeta - cos(theta1)) + (L1 + L0 * zeta) / theta1 * cos(theta0) * sin(theta1);
                double C2 = L1 / theta1 * cos(theta0) * cos(theta1) - zeta * L0 / theta1 * cos(theta0) * (1 - 1 / zeta - cos(theta1)) - (L1 + L0 * zeta) / theta1 * sin(theta0) * sin(theta1);
                Jv << (-B1 * cos(delta) / theta1 + C1 * cos(delta)) * L1 / (L1 + L0 * zeta), -B1 * sin(delta),
                    (-B1 * sin(delta) / theta1 + C1 * sin(delta))* L1 / (L1 + L0 * zeta), B1* cos(delta),
                    (-B2 / theta1 + C2)* L1 / (L1 + L0 * zeta), 0;
            }
        }
        else {
            if (std::abs(theta1) < 1e-7) {
                Jv << L1 * cos(delta) / 2, 0,
                    L1* sin(delta) / 2, 0,
                    0, 0;
            }
            else {
                double B1 = L1 / theta1 * (1 - cos(theta1));
                double B2 = L1 / theta1 * sin(theta1) + L0;
                double C1 = L1 / theta1 * sin(theta1);
                double C2 = L1 / theta1 * cos(theta1) + L0 / theta1;
                Jv << -B1 * cos(delta) / theta1 + C1 * cos(delta), -B1 * sin(delta),
                    -B1 * sin(delta) / theta1 + C1 * sin(delta), B1* cos(delta),
                    -B2 / theta1 + C2, 0;
            }
        }
    }
    return Jv;
}

Eigen::Matrix3d CalibrationMethod::S(const Eigen::Vector3d& p)
{
    Eigen::Matrix3d T;
    T << 0, -p(2), p(1),
        p(2), 0, -p(0),
        -p(1), p(0), 0;
    return T;
}

Eigen::Matrix3d CalibrationMethod::Expm(const Eigen::Vector3d& u)
{
    if (u.size() == 3) {
        double theta = u.norm();
        Eigen::Matrix3d R;

        if (theta == 0) {
            R = Eigen::Matrix3d::Identity();
        }
        else {
            Eigen::Vector3d un = u / theta;
            R = std::cos(theta) * Eigen::Matrix3d::Identity() +
                (1 - std::cos(theta)) * un * un.transpose() +
                std::sin(theta) * S(un);
        }
        return R;
    }
    else {
        return Eigen::Matrix3d::Identity(); // Handle invalid input.
    }
}

Eigen::Vector3d CalibrationMethod::calcSegP(double theta, double delta, double L)
{
    if (std::abs(theta) < 1e-7) {
        return Eigen::Vector3d(0, 0, L);
    }
    else {
        double factor = L / theta;
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);
        double cosDelta = std::cos(delta);
        double sinDelta = std::sin(delta);

        double x = factor * (cosDelta * (1 - cosTheta));
        double y = factor * (sinDelta * (1 - cosTheta));
        double z = factor * sinTheta;

        return Eigen::Vector3d(x, y, z);
    }
}

Eigen::Matrix3d CalibrationMethod::calcSegR(double theta, double delta)
{
    return Expm(Eigen::Vector3d(0, 0, delta)) * Expm(Eigen::Vector3d(0, theta, 0)) * Expm(Eigen::Vector3d(0, 0, -delta));
}

Kmp CalibrationMethod::forward(const Eigen::VectorXd& psi, const Eigen::VectorXd& seg, double zeta, double d, double ga)
{
    Kmp kmp;

    double L1x = 0;
    double l = psi(0) + d + L1x;
    double phi = psi(1);
    double theta1 = psi(2);
    double delta1 = psi(3);
    double theta2 = psi(4);
    double delta2 = psi(5);

    Eigen::VectorXd length(4);
    length << std::max(0.0, l - seg(0) - seg(1) - seg(2)),
        std::min(std::max(0.0, l - seg(1) - seg(2)), seg(0)),
        std::min(std::max(0.0, l - seg(2)), seg(1)),
        std::min(l, seg(2));

    double L0 = length(0);
    double L1 = length(1);
    double Lr = length(2);
    double L2 = length(3);
    double Lg = seg(3);

    double Theta0, Theta1;

    if (L1 < 1e-3) {
        Theta0 = 0;
        Theta1 = 0;
    }
    else {
        Theta0 = theta1 * L0 * zeta / (L1 + L0 * zeta);
        Theta1 = theta1 * L1 / (L1 + L0 * zeta);
    }

    kmp.PO_sb = Eigen::Vector3d(0, 0, -d);
    kmp.RO_sb = Expm(Eigen::Vector3d(0, 0, phi + ga / 180.0 * M_PI));

    kmp.Psb_1b = calcSegP(Theta0, delta1, L0);
    kmp.Rsb_1b = calcSegR(Theta0, delta1);

    kmp.P1b_1e = calcSegP(Theta1, delta1, L1);
    kmp.R1b_1e = calcSegR(Theta1, delta1);

    kmp.P1e_2b = Eigen::Vector3d(0, 0, Lr);
    kmp.R1e_2b = Eigen::Matrix3d::Identity();

    kmp.P2b_2e = calcSegP(theta2, delta2, L2);
    kmp.R2b_2e = calcSegR(theta2, delta2);

    kmp.P2e_g = Eigen::Vector3d(0, 0, Lg);
    kmp.R2e_g = Eigen::Matrix3d::Identity();

    kmp.Psb_g_in_sb = kmp.Psb_1b + kmp.Rsb_1b * (kmp.P1b_1e + (kmp.R1b_1e * (kmp.P1e_2b + kmp.R1e_2b * (kmp.P2b_2e + kmp.R2b_2e * kmp.P2e_g))));
    kmp.P1e_g_in_sb = kmp.Rsb_1b * kmp.R1b_1e * (kmp.P1e_2b + kmp.R1e_2b * (kmp.P2b_2e + kmp.R2b_2e * kmp.P2e_g));
    kmp.P2e_g_in_2b = kmp.R2b_2e * kmp.P2e_g;

    kmp.PO_g = kmp.PO_sb + kmp.RO_sb * (kmp.Psb_1b + kmp.Rsb_1b * (kmp.P1b_1e + kmp.R1b_1e * (kmp.P1e_2b + kmp.R1e_2b * (kmp.P2b_2e + kmp.R2b_2e * kmp.P2e_g))));
    kmp.RO_g = kmp.RO_sb * kmp.Rsb_1b * kmp.R1b_1e * kmp.R1e_2b * kmp.R2b_2e * kmp.R2e_g;

    return kmp;
}

Eigen::MatrixXd CalibrationMethod::calcJacobian(const Eigen::VectorXd& psi, const Eigen::VectorXd& seg, double zeta, double d, double ga)
{
    double L1x = 0;
    double l = psi(0) + d;
    double phi = psi(1);
    double theta1 = psi(2);
    double delta1 = psi(3);
    double theta2 = psi(4);
    double delta2 = psi(5);

    Eigen::VectorXd length(4);
    length << std::max(0.0, l - seg(0) - seg(1) - seg(2)),
        std::min(std::max(0.0, l - seg(1) - seg(2)), seg(0)),
        std::min(std::max(0.0, l - seg(2)), seg(1)),
        std::min(l, seg(2));

    double L0 = length(0);
    double L1 = length(1);
    double Lr = length(2);
    double L2 = length(3);

    Eigen::MatrixXd J(6, 6);
    J.setZero();

    Eigen::Vector3d Jphi(0, 0, 1);
    Eigen::Vector3d JwL;
    JwL.setZero();

    Eigen::Vector3d JvSeg1;
    Eigen::Vector3d JwSeg1;
    JvSeg1.setZero();
    JwSeg1.setZero();

    Eigen::Vector3d JvSeg2;
    Eigen::Vector3d JwSeg2;
    JvSeg2.setZero();
    JwSeg2.setZero();

    Eigen::MatrixXd JvL;
    JvL.setZero();

    Kmp kmp = forward(psi, seg, zeta, d, ga);

    J.block<3, 2>(0, 1) = kmp.RO_sb * (-S(kmp.Psb_g_in_sb) * Jphi);
    J.block<3, 2>(0, 1) = Jphi;

    J.block<3, 2>(0, 0) = kmp.RO_sb * (-S(kmp.P1e_g_in_sb) * JwL + JvL);
    J.block<3, 2>(0, 0) = JwL;

    J.block<3, 4>(0, 2) = kmp.RO_sb * (-S(kmp.P1e_g_in_sb) * JwSeg1 + JvSeg1);
    J.block<3, 4>(0, 2) = JwSeg1;

    J.block<3, 4>(0, 5) = kmp.RO_sb * kmp.Rsb_1b * kmp.R1b_1e * kmp.R1e_2b * (-S(kmp.P2e_g_in_2b) * JwSeg2 + JvSeg2);
    J.block<3, 4>(0, 5) = JwSeg2;

    return J;
}

Eigen::VectorXd CalibrationMethod::calcPsiFromQ(const Eigen::VectorXd& qa, const Eigen::VectorXd& psi_in, const Eigen::VectorXd& x)
{
    double l = psi_in(0) / 1000;
    Eigen::VectorXd config = psi_in;

    double l1, ls;

    if (l - mp.L2 - mp.Lr + mp.d + mp.L1x > mp.L1) {
        l1 = mp.L1;
        ls = l - mp.L2 - mp.Lr - mp.L1 + mp.d + mp.L1x;
    }
    else {
        l1 = l - mp.L2 - mp.Lr + mp.d + mp.L1x;
        ls = 0;
    }

    Eigen::Matrix3d K1, K2;
    K1.setZero();
    K2.setZero();
    K1.diagonal() << x(6), x(7), 1;
    K2.diagonal() << x(8), x(9), 1;

    Eigen::MatrixXd MatP = Eigen::MatrixXd::Zero(12, 9);
    Eigen::MatrixXd MatL = Eigen::MatrixXd::Zero(12, 12);
    Eigen::MatrixXd MatA = Eigen::MatrixXd::Zero(9, 12);
    Eigen::MatrixXd MatK = Eigen::MatrixXd::Zero(9, 9);

    MatP.block(0, 0, 2, 3) = (l1 + ls * mp.zeta) * mp.Q1.transpose();
    MatP.block(2, 0, 8, 3) = (l1 + ls * mp.zeta) * mp.Q2.transpose();
    MatP.block(2, 3, 8, 3) = mp.L2 * mp.Q2.transpose();
    MatP.block(2, 6, 8, 3) = -mp.alpha * mp.L3 * mp.Q2.transpose();
    MatP.block(10, 6, 2, 3) = -mp.L3 * mp.Q3.transpose();

    MatL.block(0, 0, 2, 2).diagonal().array() = (mp.L1 + mp.La);
    MatL.block(2, 2, 8, 8).diagonal().array() = (mp.L2 + mp.Lr + mp.L1 + mp.La + mp.Lc + mp.L3);
    MatL.block(10, 10, 2, 2).diagonal().array() = mp.L3;

    MatA.block(0, 0, 3, 2) = mp.A1 * mp.E * mp.Q1;
    MatA.block(3, 2, 6, 8) = mp.A2 * mp.E * mp.Q2;
    MatA.block(7, 10, 2, 2) = mp.A3 * mp.E * mp.Q3;

    MatK.block(0, 0, 3, 3) = 4 * mp.Kb1 + 16 * mp.Kb2 + K1 * mp.Kb1;
    MatK.block(0, 3, 3, 3) = -16 * mp.Kb2 - K2 * mp.Kb1;
    MatK.block(3, 3, 3, 3) = 16 * mp.Kb2 + K2 * mp.Kb1;
    MatK.block(6, 3, 3, 3) = mp.alpha * (16 * mp.Kb2 + K2 * mp.Kb1);
    MatK.block(6, 6, 3, 3) = 4 * mp.Kb3 + 16 * mp.Kb2;

    Eigen::VectorXd u = pinv(MatP + MatL * pinv(MatA) * MatK) * qa;

    Eigen::VectorXd u1 = u.segment(0, 3);
    Eigen::VectorXd u2 = u.segment(3, 3);
    Eigen::VectorXd u3 = u.segment(6, 3);

    config(2) = u1.norm() * (ls * mp.zeta + l1);
    config(3) = -M_PI / 2 + std::atan2(u1(1), u1(0));
    config(4) = mp.L2 * u2.norm();
    config(5) = -M_PI / 2 + std::atan2(u2(1), u2(0));
    config = limitPsiNum(config);

    return config;
}

Eigen::VectorXd CalibrationMethod::limitPsiNum(Eigen::VectorXd psi)
{
    if (psi(2) < 0) {
        psi(2) = -psi(2);
        psi(3) = psi(3) + M_PI;
    }
    if (psi(4) < 0) {
        psi(4) = -psi(4);
        psi(5) = psi(5) + M_PI;
    }

    while (psi(3) > 2 * M_PI) {
        psi(3) = psi(3) - 2 * M_PI;
    }
    while (psi(3) < -2 * M_PI) {
        psi(3) = psi(3) + 2 * M_PI;
    }
    while (psi(5) > 2 * M_PI) {
        psi(5) = psi(5) - 2 * M_PI;
    }
    while (psi(5) < -2 * M_PI) {
        psi(5) = psi(5) + 2 * M_PI;
    }

    return psi;
}

Eigen::VectorXd CalibrationMethod::calcQFrompsi(Eigen::VectorXd psi, Eigen::VectorXd x)
{
    double l = psi(0) / 1000;
    Eigen::VectorXd qa(12);
    Eigen::VectorXd qa_extend(12);
    double l1(0), ls(0);
    if (l - mp.L2 - mp.Lr + mp.d + mp.L1x > mp.L1) {
        l1 = mp.L1;
        ls = l - mp.L2 - mp.Lr - mp.L1 + mp.d + mp.L1x;
    }
    else {
        l1 = l - mp.L2 - mp.Lr + mp.d + mp.L1x;
        ls = 0;
    }

    double theta1 = psi(2) * l1 / (ls * mp.zeta + l1);
    double thetas = psi(2) * ls * mp.zeta / (ls * mp.zeta + l1);
    double delta1 = psi(3);
    double theta2 = psi(4);
    double delta2 = psi(5);

    Eigen::VectorXd u1(3);
    u1 << theta1 / l1 * cos(delta1 + M_PI / 2), theta1 / l1 * sin(delta1 + M_PI / 2), 0;

    Eigen::VectorXd u2(3);
    u2 << theta2 / mp.L2 * cos(delta2 + M_PI / 2), theta2 / mp.L2 * sin(delta2 + M_PI / 2), 0;

    Eigen::MatrixXd K1(3, 3);
    K1.setZero();
    K1(0, 0) = x(5);
    K1(1, 1) = x(6);
    K1(2, 2) = 1.0;

    Eigen::MatrixXd K2(3, 3);
    K2.setZero();
    K2(0, 0) = x(7);
    K2(1, 1) = x(8);
    K2(2, 2) = 1.0;

    Eigen::MatrixXd MatP(12, 9);
    MatP.setZero();
    MatP.block(0, 0, 2, 3) = (l1 + ls * mp.zeta) * mp.Q1;
    MatP.block(2, 0, 8, 3) = (l1 + ls * mp.zeta) * mp.Q2;
    MatP.block(2, 3, 8, 3) = mp.L2 * mp.Q2;
    MatP.block(2, 6, 8, 3) = -mp.alpha * mp.L3 * mp.Q2;
    MatP.block(10, 6, 2, 3) = -mp.L3 * mp.Q3;

    Eigen::MatrixXd MatL(12, 12);
    MatL.setZero();
    MatL.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * (mp.L1 + mp.La);
    MatL.block(2, 2, 8, 8) = Eigen::MatrixXd::Identity(8, 8) * (mp.L2 + mp.Lr + mp.L1 + mp.La + mp.Lc + mp.L3);
    MatL.block(10, 10, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * mp.L3;

    Eigen::MatrixXd MatA(9, 12);
    MatA.setZero();
    MatA.block(0, 0, 3, 2) = mp.A1 * mp.E * mp.Q1;
    MatA.block(3, 0, 6, 8) = mp.A2 * mp.E * mp.Q2;
    MatA.block(6, 0, 3, 2) = mp.A3 * mp.E * mp.Q3;

    Eigen::MatrixXd MatK(9, 9);
    MatK.setZero();
    MatK.block(0, 0, 3, 3) = 4 * mp.Kb1 + 16 * mp.Kb2 + K1 * mp.Kb1;
    MatK.block(0, 3, 3, 3) = -16 * mp.Kb2 - K2 * mp.Kb1;
    MatK.block(3, 3, 3, 3) = 16 * mp.Kb2 + K2 * mp.Kb1;
    MatK.block(6, 3, 3, 3) = mp.alpha * (16 * mp.Kb2 + K2 * mp.Kb1);
    MatK.block(6, 6, 3, 3) = 4 * mp.Kb3 + 16 * mp.Kb2;

    Eigen::VectorXd u3(3);
    u3 = (mp.Q2.transpose() / (mp.alpha * mp.L3)) * ((mp.Q2.transpose() * (l1 + mp.zeta * ls) * u1) +
        (mp.Q2.transpose() * mp.L2 + ((mp.L2 + mp.Lr + mp.L1 + mp.La + mp.Lc + mp.L3) / (mp.A2 * mp.E)) * (16 * mp.Kb2 + K2 * mp.Kb1)) * u2);

    Eigen::VectorXd u(9);
    u << u1, u2, u3;

    qa_extend = (MatP + MatL * pinv(MatA) * MatK).inverse() * u * 1000;

    for (int i = 0; i < 12; i++) {
        qa(i) = qa_extend(i);
    }

    return qa;
}

Eigen::MatrixXd CalibrationMethod::pinv(const Eigen::MatrixXd& mat)
{
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

    const auto& singlularValues = svd.singularValues();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    double pinvtoler = 1E-8;
    for (unsigned int i = 0; i < singlularValues.size(); i++) {
        if (singlularValues(i) > pinvtoler)
            singularValuesInv(i, i) = 1.0f / singlularValues(i);
        else
            singularValuesInv(i, i) = 0.0f;
    }
    Eigen::MatrixXf pinvmat = svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
    return pinvmat;
}

Eigen::Matrix3d CalibrationMethod::EulerToRotMat(const Eigen::Vector3d& euler)
{
    double r = euler(0);
    double p = euler(1);
    double y = euler(2);

    Eigen::Matrix3d R = Expm(Eigen::Vector3d(0, 0, 1) * (y * M_PI / 180)) *
        Expm(Eigen::Vector3d(0, 1, 0) * (p * M_PI / 180)) *
        Expm(Eigen::Vector3d(1, 0, 0) * (r * M_PI / 180));

    return R;
}

std::vector<Eigen::Matrix4d> CalibrationMethod::forwardCalib(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Psi_ref, const MP& x)
{
    std::vector<Eigen::Matrix4d> Poses;
    //Eigen::VectorXd seg(4);
    ////seg << x(3), x(4), x(5), 15;

    ////double ga = x(1);

    //for (int i = 0; i < Q.cols(); ++i) {
    //    Eigen::VectorXd psi_i = calcPsiFromQ(Q.col(i), Psi_ref.col(i), x);
    //    Eigen::Matrix4d Pose_i;
    //    forward(Pose_i, psi_i, seg, 0.15, 8, ga, x);
    //    Poses.push_back(Pose_i);
    //}

    return Poses;
}


