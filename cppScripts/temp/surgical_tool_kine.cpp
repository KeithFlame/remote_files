#include "../include/define/surgical_tool_kine.h"


void calcForwardKinematics(const ConfigSpcs& qs, TaskSpc& task_space)
{
    mmath::Pose pose;
    for(int i = 0; i < qs.size(); i++){
        auto& q = qs[i];
        mmath::continuum::calcSingleSegmentPose(q, pose);
        task_space.add(pose);
        task_space.end2base *= pose;
    }
}


TaskSpc calcForwardKinematics(const ConfigSpcs& qs)
{
    TaskSpc task_space;
    calcForwardKinematics(qs, task_space);
    return task_space;
}


bool calcInverseKinematicsC3(const mmath::Pose &pose,
                             const SurgicalToolParam& param,
                             SurgicalToolConfig &config)
{
    bool flag = true;

    float r1_min = 10; // Determined by tool mechanism
    //float theta1_max = config.theta1_max;
    float theta2_max = config.theta2_max;
    float L1_max = param.getL1();
    float Lr = param.getLr();
    float L2 = param.getL2();
    float Lg = param.getLg();

    // Ref Eq(3)
    //float l1_max = L1_max / theta1_max * tanf(theta1_max / 2);
    float l2_max = L2 / theta2_max * tanf(theta2_max / 2);

    Eigen::Vector3f n, a;
    n = pose.R.col(0);
    a = pose.R.col(2);
    // End position
    float pex = pose.t[0], pey = pose.t[1], pez = pose.t[2];
    float ptx = sqrtf(pex * pex + pey * pey);
    float ptz = pez;

    // Rotate (there is also a rotate-back behind)
    float angle = atan2f(pey, pex);

    Eigen::Matrix3f R = mmath::rotByZf(angle);
    Eigen::Vector3f ar = R.transpose() * a;
    float ax0 = ar[0], ay0 = ar[1], az0 = ar[2];
    float ax = ax0, ay = ay0, az = az0;

    /*
     * Determine if theta2 is within limit and solve theta2
     */

    // Talor series expansion
    auto TalorSeriesExpansion = [&L2](float theta2, float &l2, float &dl2) {
        if(theta2 <= 1e-4){
            l2 = L2 * (1.f/2 + pow(theta2,2) / 24.f);
            dl2 = L2 * (theta2 / 6 - pow(theta2,3) / 120) / (cos(theta2) + 1);
        }
        else{
            l2 = L2 / theta2 * tanf(theta2 / 2);
            dl2 = L2 * (theta2 - sin(theta2))/((cos(theta2) + 1)*powf(theta2,2));
        }
    };

    float t2 = 0, l2 = L2 / 2;
    if ((ptx*ptx + ptz*ptz) <= powf(Lr + L2 + Lg, 2)){
        t2 = M_PI / 3; // initial guess
        float dl2 = 0;
        TalorSeriesExpansion(t2, l2, dl2);
        float ft = powf(Lg + l2, 2) + powf(Lr + l2, 2) +
                2*(Lg + l2)*(Lr + l2)*cos(t2) - ptx*ptx - ptz*ptz;
        float thresh = 1e-15;
        int k = 0;
        while(abs(ft) > thresh && k < 10){
            k++;
            float dft = 2*(Lg + Lr + 2*l2) * (1 + cos(t2))*dl2 -
                    2*(Lg + l2)*(Lr + l2)*sin(t2);
            int mt = -1;
            float fta = ft, t2a = 0;
            while (abs(fta) >= abs(ft) && mt < 10){
                mt++;
                t2a = t2 - ft / (dft*pow(2, mt));
                t2a = std::min(theta2_max, std::max(t2a, 0.f));
                TalorSeriesExpansion(t2a, l2, dl2);
                fta = pow(Lg + l2, 2) + pow(Lr + l2, 2) +
                        2*(Lg + l2)*(Lr + l2)*cos(t2a) - ptx*ptx - ptz*ptz;
            }
            ft = fta;
            t2 = t2a;
        }
    }
    float theta2 = 0;

    auto eqn = [&Lr, &Lg, &ptx, &ptz, &ax, &az](float theta2, float l2,
            float &AA, float &BB, float &CC) -> float
    {
        float c1 = l2 + Lr + ptz;
        float c2 = l2 + Lg;
        float c3 = c1*c1 + c2*c2;
        float c4 = c1*c2;
        AA = -2*ptx*(c1 + c2*cos(theta2));
        BB = -(c3 - ptx*ptx + 2*c4*cos(theta2));
        CC = 2*c4 + (c3 + ptx*ptx)*cos(theta2);
        return AA*ax + BB*az + CC;
    };
    auto ABCD = [&Lr, &Lg, &ptx, &ptz](float ax, float ay, float az,
            float &A, float &B, float &C, float &D, float &pz) -> float
    {
        float px = ptx - ax*Lg;
        float py = -ay*Lg;
        pz = ptz - az*Lg;
        float Pa = px*ax + py*ay + pz*az;
        A = -2.f*(Pa + Lr);
        B = px*px + py*py + pz*pz - Lr*Lr;
        C = 2.f*(1 - az);
        D = 2.f*(pz + Lr);
        return Pa;
    };
    float A = 0, B = 0, C = 0, D = 0, pz = 0;

    float A12 = 0, B12 = 0, C12 = 0, A10 = 0, B10 = 0, C10 = 0;
    float eqncost2 = eqn(theta2_max, l2_max, A12, B12, C12);
    float eqnL0 = eqn(t2, l2, A10, B10, C10);

    if (eqncost2 > 0) { // theta2>theta2m
        float S12 = A12*A12 + B12*B12;
        ax = (sqrt((S12 - C12*C12) / (S12 - pow(A12*ax0 + B12*az0, 2)))*
              (B12*B12*ax0 - A12*B12*az0) - A12*C12) / S12;
        az = -(A12*ax + C12) / B12;
        std::complex<float> tmp = sqrt(std::complex<float>(1 - ax*ax - az*az, 0));
        ay = abs(ay) <= 1e-7 ? tmp.real() : ay / abs(ay) * tmp.real();
        ABCD(ax, ay, az, A, B, C, D, pz);

        theta2 = theta2_max;
        l2 = l2_max;
        flag = false;
    }
    else if (eqnL0 < 0) { // L1 < 0
        float S10 = A10*A10 + B10*B10;
        ax = (sqrt((S10 - C10*C10) / (S10 - pow(A10*ax0 + B10*az0, 2)))*
              (B10*B10*ax0 - A10*B10*az0) - A10*C10) / S10;
        az = -(A10*ax + C10) / B10;
        std::complex<float> tmp = sqrt(std::complex<float>(1 - ax*ax - az*az, 0));
        ay = abs(ay) <= 1e-7 ? tmp.real() : ay / abs(ay) * tmp.real();
        ABCD(ax, ay, az, A, B, C, D, pz);

        theta2 = t2;
        flag = false;
    }
    else{ // theta2 <= theta2m, iteration will converge within limit
        float Pa = ABCD(ax, ay, az, A, B, C, D, pz);
        float f1 = A + D + C*Lr;
        float f2 = D + A*az - Pa*C;
        float f3 = B + D*Lr;
        float f4 = az*B - Pa*D;

        // Solve theta2
        theta2 = M_PI / 3; // initial guess
        float dl2 = 0;
        TalorSeriesExpansion(theta2, l2, dl2);
        float f = C*l2*l2*(cos(theta2) + 1) + f1*cos(theta2)*l2 + f2*l2 +
                f3*cos(theta2) + f4;

        float thresh = 3e-4; // 4.5 5e-2 5e-3 5e-4 1e-4 1e-5 1e-6
        int k = 0;
        while (abs(f) > thresh && k < 20) { // iteration number limit
            k++;
            float df = C*(2*l2*dl2*(cos(theta2) + 1) - l2*l2*sin(theta2)) +
                    f1*(cos(theta2)*dl2 - sin(theta2)*l2) + f2*dl2 -
                    f3*sin(theta2);
            theta2 = std::min(theta2_max,std::max(theta2 - f/df, 0.f));

            TalorSeriesExpansion(theta2, l2, dl2);
            f = C*l2*l2*(cos(theta2) + 1) + f1*cos(theta2)*l2 + f2*l2 +
                    f3*cos(theta2) + f4;
        }
    }
    config.set(theta2, CONFIG_THETA2);

    /*
     * Solve theta1, L1
     */
    float l1 = (A*l2 + B) / (C*l2 + D);

    float costheta1 = (pz - az*l2 - l1) / (l1 + l2 + Lr);
    if (costheta1 > 1)  costheta1 = 1;
    float theta1 = acos(costheta1);

    float L1 = 0;
    if (theta1 <= 1e-4) L1 = l1 / (1.f / 2 + theta1*theta1 / 24);
    else                L1 = l1*theta1 / tan(theta1 / 2.f);

    config.set(theta1, CONFIG_THETA1);
    config.set(L1, CONFIG_L_INSERT);

    /*
     * out of dexterous workspace
     * To be completed!!!
     */
    if (L1 < r1_min*theta1 - 0.1 || L1 > L1_max + 0.1){
        flag = false; // r1 lim
    }

    // rotate back
    ar = R*Eigen::Vector3f(ax, ay, az);
    ax = ar[0];    ay = ar[1];    az = ar[2];
    float px = pex - ax*Lg;
    float py = pey - ay*Lg;
    pz = pez - az*Lg;

    // delta1 + phi
    float delta1_plus_phi = 0;
    if (theta1 > 1e-4) {
        delta1_plus_phi = atan2(py - ay*l2, px - ax*l2);
    }

    mmath::Pose pose2;
    mmath::continuum::calcSingleSegmentPose(0, theta1, delta1_plus_phi, pose2);

    // delta2 + phi
    float delta2_plus_phi = 0;
    if (theta2 > 1e-4) {
        float k = 0;
        if (theta1 < 1e-4) {
            k = L1*(theta1/2.f - pow(theta1, 3)/24);
        }
        else{
            k = (1 - costheta1) * L1/theta1;
        }
        Eigen::Vector3f p3 = pose2.R * Eigen::Vector3f(px, py, -pz);
        float p3x = p3[0] + k*cos(delta1_plus_phi);
        float p3y = p3[1] + k*sin(delta1_plus_phi);

        delta2_plus_phi = atan2(p3y, p3x);
    }

    // phi
    mmath::Pose pose4;
    mmath::continuum::calcSingleSegmentPose(0, theta2, delta2_plus_phi, pose4);

    Eigen::Matrix3f R04 = pose2.R*pose4.R;
    Eigen::Vector3f n4 = R04.transpose()*n;
    float phi = atan2(n4(1),n4(0));

    // delta1 and delta2
    float delta1 = delta1_plus_phi - phi;
    float delta2 = delta2_plus_phi - phi;

    config.set(phi, CONFIG_PHI);
    config.set(delta1, CONFIG_DELTA1);
    config.set(delta2, CONFIG_DELTA2);

    return flag;
}
