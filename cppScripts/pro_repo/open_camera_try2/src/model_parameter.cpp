#include "model_parameter.h"

MP::MP()
{
    e1 << 1, 0, 0;
    e2 << 0, 1, 0;
    e3 << 0, 0, 1;
    E = 50e9;
    mu = 0.25;
    d1 = 0.95e-3;
    d2 = 0.4e-3;
    d3 = 0.95e-3;
    rho1 = 2.5e-3;
    rho2 = 2.7e-3;
    rhop = 10e-3 * 1e-3;
    rho3 = 8e-3;
    alpha = rhop / rho2;
    L1 = 3 * 1e-3;
    L2 = 5 * 1e-3;
    L3 = 20e-3;
    Lc = 64.5e-3;
    La = 2 * 1e-3;
    Lr = 4 * 1e-3;
    Lg = 15e-3;
    r11 = e1 * rho1;
    r12 = e2 * rho1;
    r13 = -e1 * rho1;
    r14 = -e2 * rho1;
    r21 << cos(21 / 180.0 * M_PI), sin(21 / 180.0 * M_PI), 0;
    r22 << cos(37 / 180.0 * M_PI), sin(37 / 180.0 * M_PI), 0;
    r23 << cos(53 / 180.0 * M_PI), sin(53 / 180.0 * M_PI), 0;
    r24 << cos(69 / 180.0 * M_PI), sin(69 / 180.0 * M_PI), 0;
    r25 << cos(111 / 180.0 * M_PI), sin(111 / 180.0 * M_PI), 0;
    r26 << cos(127 / 180.0 * M_PI), sin(127 / 180.0 * M_PI), 0;
    r27 << cos(143 / 180.0 * M_PI), sin(143 / 180.0 * M_PI), 0;
    r28 << cos(159 / 180.0 * M_PI), sin(159 / 180.0 * M_PI), 0;
    r29 << cos(201 / 180.0 * M_PI), sin(201 / 180.0 * M_PI), 0;
    r2a << cos(217 / 180.0 * M_PI), sin(217 / 180.0 * M_PI), 0;
    r2b << cos(233 / 180.0 * M_PI), sin(233 / 180.0 * M_PI), 0;
    r2c << cos(249 / 180.0 * M_PI), sin(249 / 180.0 * M_PI), 0;
    r2d << cos(291 / 180.0 * M_PI), sin(291 / 180.0 * M_PI), 0;
    r2e << cos(307 / 180.0 * M_PI), sin(307 / 180.0 * M_PI), 0;
    r2f << cos(323 / 180.0 * M_PI), sin(323 / 180.0 * M_PI), 0;
    r2g << cos(339 / 180.0 * M_PI), sin(339 / 180.0 * M_PI), 0;
    r31 = e1 * rho3;
    r32 = e2 * rho3;
    r33 = -e1 * rho3;
    r34 = -e2 * rho3;
    Q1 << S(r11) * e3, S(r12)* e3, S(r13)* e3;
    Q2 << S(r21) * e3, S(r22)* e3, S(r23)* e3, S(r24)* e3,
        S(r25)* e3, S(r26)* e3, S(r27)* e3, S(r28)* e3;
    Q3 << S(r31) * e3, S(r32)* e3, S(r33)* e3, S(r34)* e3;
    I1 = M_PI * std::pow(d1, 4) / 64;
    A1 = M_PI * std::pow(d1, 2) / 4;
    I2 = M_PI * std::pow(d2, 4) / 64;
    A2 = M_PI * std::pow(d2, 2) / 4;
    I3 = M_PI * std::pow(d3, 4) / 64;
    A3 = M_PI * std::pow(d3, 2) / 4;
    G = E / 2 / (1 + mu);
    J1 = 2 * I1;
    J2 = 2 * I2;
    J3 = 2 * I3;
    Ke1 = G * A1 * Eigen::Matrix3d::Identity();
    Ke2 = G * A2 * Eigen::Matrix3d::Identity();
    Ke3 = G * A3 * Eigen::Matrix3d::Identity();
    Kb1 = E * I1 * Eigen::Matrix3d::Identity();
    Kb2 = E * I2 * Eigen::Matrix3d::Identity();
    Kb3 = E * I3 * Eigen::Matrix3d::Identity();
    zeta = 0.15;
    L1x = 0;
    d = 8e-3;
}

MP MP::resetMP(Eigen::VectorXd)
{
    return MP();
}

Eigen::VectorXd MP::getMP()
{
    return Eigen::VectorXd();
}

Eigen::Matrix3d MP::S(const Eigen::Vector3d& p)
{
    Eigen::Matrix3d T;
    T << 0, -p(2), p(1),
        p(2), 0, -p(0),
        -p(1), p(0), 0;
    return T;
}
