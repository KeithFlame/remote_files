#include "base_info.h"
#include <type_traits>
#include "tinyxml.h"
#include <iostream>

ToolBaseInfo::ToolBaseInfo() :
	//关节限值
	l_max_limit(0.f),
	l_min_limit(200.f),
	phi_limit(4.f),
	theta1_limit(1.5708f),
	theta2_limit(1.5708f),

	//结构尺寸----连续体各段长度
	L1(100.f),
	Lr(10.f),
	L2(20.f),
	Lg(15.f),

	//结构尺寸2----用于从关节空间计算到构型空间
	Lstem(381.5f),
	zeta(0.1f),
	gamma_1(-0.7854f),
	gamma_3(0.f),
	L1x(-1.f),
	d(-10.f),
	K1(5.f),
	K2(0.6f),

	//结构尺寸3---用于计算耦合矩阵
	Lcnla(200.f),
	Lprox(18.f),
	d1(0.95f),
	d2(0.4f),
	E(50e9f),
	poisson_ratio(0.25f),
	r1(2.5f),
	r2(2.7f),
	r3(12.f),

	// 模型计算
	epsilon_position(1.f),
	epsilon_direction(1.f),
	epsilon_force(1.f),
	epsilon_moment(1.f),

	//位置计算
	v_limit(100.f),
	w_limit(1.5708f),
	v_limit_thred(5.f),
	dt(0.001f),
	//臂体信息
	arm_name("手术工具1"),
	arm_serial("10001")
{

}

ToolBaseInfo::ToolBaseInfo(std::string path)
{
	bool rc = readXml(path);	
}

bool ToolBaseInfo::getLMaxLimit(float& l_max_limit)
{
	l_max_limit = this->l_max_limit;
	return true;
}

bool ToolBaseInfo::getLMinLimit(float& l_min_limit)
{
	l_min_limit = this->l_min_limit;
	return true;
}

bool ToolBaseInfo::getPhiLimit(float& phi_limit)
{
	phi_limit = this->phi_limit;
	return false;
}

bool ToolBaseInfo::getTheta1Limit(float& theta1_limit)
{
	theta1_limit = this->theta1_limit;
	return true;
}

bool ToolBaseInfo::getTheta2Limit(float& theta2_limit)
{
	theta2_limit = this->theta2_limit;
	return true;
}

bool ToolBaseInfo::getL1(float& L1)
{
	L1 = this->L1;
	return true;
}

bool ToolBaseInfo::getLr(float& Lr)
{
	Lr = this->Lr;
	return true;
}

bool ToolBaseInfo::getL2(float& L2)
{
	L2 = this->L2;
	return true;
}

bool ToolBaseInfo::getLg(float& Lg)
{
	Lg = this->Lg;
	return true;
}

bool ToolBaseInfo::getLstem(float& Lstem)
{
	Lstem = this->Lstem;
	return true;
}

bool ToolBaseInfo::getZeta(float& zeta)
{
	zeta = this->zeta;
	return true;
}

bool ToolBaseInfo::getgamma_1(float& gamma_1)
{
	gamma_1 = this->gamma_1;
	return true;
}

bool ToolBaseInfo::getgamma_3(float& gamma_3)
{
	gamma_3 = this->gamma_3;
	return true;
}

bool ToolBaseInfo::getL1x(float& L1x)
{
	L1x = this->L1x;
	return true;
}

bool ToolBaseInfo::getd(float& d)
{
	d = this->d;
	return true;
}

bool ToolBaseInfo::getd1(float& d1)
{
	d1 = this->d1;
	return true;
}

bool ToolBaseInfo::getd2(float& d2)
{
	d2 = this->d2;
	return true;
}

bool ToolBaseInfo::getE(float& E)
{
	E = this->E;
	return true;
}

bool ToolBaseInfo::getPoisson_ratio(float& poisson_ratio)
{
	poisson_ratio = this->poisson_ratio;
	return true;
}

bool ToolBaseInfo::getK1(float& K1)
{
	K1 = this->K1;
	return true;
}

bool ToolBaseInfo::getK2(float& K2)
{
	K2 = this->K2;
	return true;
}

bool ToolBaseInfo::getLcnla(float& Lcnla)
{
	Lcnla = this->Lcnla;
	return true;
}

bool ToolBaseInfo::getLprox(float& Lprox)
{
	Lprox = this->Lprox;
	return true;
}

bool ToolBaseInfo::getR1(float& r1)
{
	r1 = this->r1;
	return true;
}

bool ToolBaseInfo::getR2(float& r2)
{
	r2 = this->r2;
	return true;
}

bool ToolBaseInfo::getR3(float& r3)
{
	r3 = this->r3;
	return false;
}

bool ToolBaseInfo::getEpsilon_position(float& epsilon_position)
{
	epsilon_position = this->epsilon_position;
	return true;
}

bool ToolBaseInfo::getEpsilon_direction(float& epsilon_direction)
{
	epsilon_direction = this->epsilon_direction;
	return true;
}

bool ToolBaseInfo::getEpsilon_force(float& epsilon_force)
{
	epsilon_force = this->epsilon_force;
	return true;
}

bool ToolBaseInfo::getEpsilon_moment(float& epsilon_moment)
{
	epsilon_moment = this->epsilon_moment;
	return true;
}

bool ToolBaseInfo::getVLimit(float& v_limit)
{
	v_limit = this->v_limit;
	return true;
}

bool ToolBaseInfo::getWLimit(float& w_limit)
{
	w_limit = this->w_limit;
	return true;
}

bool ToolBaseInfo::getVLimitThreshold(float& v_limit_thred)
{
	v_limit_thred = this->v_limit_thred;
	return true;
}

bool ToolBaseInfo::getDt(float& dt)
{
	dt = this->dt;
	return true;
}

bool ToolBaseInfo::getArmName(std::string& arm_name)
{
	arm_name = this->arm_name;
	return true;
}

bool ToolBaseInfo::getArmSerial(std::string& arm_serial)
{
	arm_serial = this->arm_serial;
	return true;
}

bool ToolBaseInfo::readXml(std::string path)
{
	TiXmlDocument doc;
	if (!doc.LoadFile(path.c_str()))   //检测xml文档是否存在
	{
		std::cerr << doc.ErrorDesc() << std::endl;
		return false;
	}
	TiXmlElement* tool_info = doc.FirstChildElement();//指向xml文档的根


	if (tool_info == NULL)//检测根元素的存在     
	{
		std::cerr << "Failed to load file: No root element." << std::endl;
		doc.Clear();
		return false;
	}
	else
	{
		TiXmlElement* arm_name_id = tool_info->FirstChildElement(); // xml的id 根节点下第一个子节点
		TiXmlElement* arm_info_id = arm_name_id->NextSiblingElement();

		arm_name = arm_name_id->GetText();
		arm_serial = arm_info_id->GetText();
		auto upload_value = [&arm_info_id]()->float
		{
			arm_info_id = arm_info_id->NextSiblingElement();
			return std::stof(arm_info_id->GetText());
		};

		//结构尺寸----连续体各段长度
		L1 = upload_value();
		Lr = upload_value();
		L2 = upload_value();
		Lg = upload_value();

		//关节限值
		l_max_limit = upload_value();
		l_min_limit = upload_value();
		phi_limit = upload_value();
		theta1_limit = upload_value();
		theta2_limit = upload_value();

		//结构尺寸2----用于从关节空间计算到构型空间
		Lstem = upload_value();
		zeta = upload_value();
		gamma_1 = upload_value();
		gamma_3 = upload_value();
		L1x = upload_value();
		d = upload_value();
		K1 = upload_value();
		K2 = upload_value();

		// 结构尺寸3---用于计算耦合矩阵
		Lcnla = upload_value();
		Lprox = upload_value();
		d1 = upload_value();
		d2 = upload_value();
		E = upload_value();
		poisson_ratio = upload_value();
		r1 = upload_value();
		r2 = upload_value();
		r3 = upload_value();

		// 模型计算
		epsilon_position = upload_value();
		epsilon_direction = upload_value();
		epsilon_force = upload_value();
		epsilon_moment = upload_value();

		// 位姿计算
		v_limit = upload_value();
		w_limit = upload_value();
		v_limit_thred = upload_value();
		dt = upload_value();

		/*
		arm_info_id = arm_info_id->NextSiblingElement();
		L1 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		Lr = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		L2 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		Lg = std::stof(arm_info_id->GetText());

		arm_info_id = arm_info_id->NextSiblingElement();
		l_max_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		l_min_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		phi_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		theta1_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		theta2_limit = std::stof(arm_info_id->GetText());

		arm_info_id = arm_info_id->NextSiblingElement();
		Lstem = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		zeta = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		gamma_1 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		gamma_3 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		L1x = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		d = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		K1 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		K2 = std::stof(arm_info_id->GetText());

		arm_info_id = arm_info_id->NextSiblingElement();
		Lcnla = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		Lprox = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		d1 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		d2 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		E = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		poisson_ratio = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		r1 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		r2 = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		r3 = std::stof(arm_info_id->GetText());

		arm_info_id = arm_info_id->NextSiblingElement();
		epsilon_position = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		epsilon_direction = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		epsilon_force = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		epsilon_moment = std::stof(arm_info_id->GetText()); 

		arm_info_id = arm_info_id->NextSiblingElement();
		v_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		w_limit = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		v_limit_thred = std::stof(arm_info_id->GetText());
		arm_info_id = arm_info_id->NextSiblingElement();
		dt = std::stof(arm_info_id->GetText());
		*/
	}

	
	return true;
}
