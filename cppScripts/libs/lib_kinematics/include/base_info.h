#pragma once
#ifndef BASE_INFO_H_
#define BASE_INFO_H_
#include <string>
class ToolBaseInfo
{
public:
	ToolBaseInfo();
	ToolBaseInfo(ToolBaseInfo&) = default;
	ToolBaseInfo(std::string);

	bool getLMaxLimit(float&);
	bool getLMinLimit(float&);
	bool getPhiLimit(float&);
	bool getTheta1Limit(float&);
	bool getTheta2Limit(float&);
	
	bool getL1(float&);
	bool getLr(float&);
	bool getL2(float&);
	bool getLg(float&);

	bool getLstem(float&);
	bool getZeta(float&);
	bool getgamma_1(float&);
	bool getgamma_3(float&);
	bool getL1x(float&);
	bool getd(float&);
	bool getK1(float&);
	bool getK2(float&);

	bool getLcnla(float&);
	bool getLprox(float&);
	bool getd1(float&);
	bool getd2(float&);
	bool getE(float&);
	bool getPoisson_ratio(float&);
	bool getR1(float&);
	bool getR2(float&);
	bool getR3(float&);

	bool getEpsilon_position(float&);
	bool getEpsilon_direction(float&);
	bool getEpsilon_force(float&);
	bool getEpsilon_moment(float&);

	bool getVLimit(float&);
	bool getWLimit(float&);
	bool getVLimitThreshold(float&);
	bool getDt(float&);

	bool getArmName(std::string&);
	bool getArmSerial(std::string&);

private:

private:

	bool readXml(std::string);
	//�ؽ���ֵ
	float l_max_limit;
	float l_min_limit;
	float phi_limit;
	float theta1_limit;
	float theta2_limit;

	//�ṹ�ߴ�----��������γ���
	float L1;
	float Lr;
	float L2;
	float Lg;
	
	//�ṹ�ߴ�2----���ڴӹؽڿռ���㵽���Ϳռ�
	float Lstem;
	float zeta;
	float gamma_1;
	float gamma_3;
 	float L1x;
	float d;
	float K1;
	float K2;
	
	// �ṹ�ߴ�3---���ڼ�����Ͼ���
	float Lcnla;
	float Lprox;
	float d1;
	float d2;
	float E;
	float poisson_ratio;
	float r1;
	float r2;
	float r3;

	// ģ�ͼ���
	float epsilon_position;
	float epsilon_direction;
	float epsilon_force;
	float epsilon_moment;
	
	// λ�˼���
	float v_limit;
	float w_limit;
	float v_limit_thred;
	float dt;
	 
	// ��������
	std::string arm_name;
	std::string arm_serial;

};
#endif // !BASE_INFO_H_
