#include "continuum_segment.h"
#include <iostream>

ContinuumSegment::ContinuumSegment():
	E(40e9f)
	,mu(0.33f)
	,d(0.95e-3f)
	,L(0.1f)
	,rho(2.5e-3f)
	,delta_t(0.f)
	,K(5.f)
	,gamma(0.f)
	,L0(10e-3f)
{
	G = E / (1 + mu) / 2.f;
	refreshR12();
	refreshStiffness();
}

ContinuumSegment::ContinuumSegment(ContinuumSegment& seg)
{
	E = seg.getE();
	mu = seg.getMu();
	d = seg.getD();
	L = seg.getL();
	rho = seg.getRho();
	delta_t = seg.getDelta_t();
	K = seg.getK();
	gamma = seg.getGamma();
	L0 = seg.getL0();
	G = E / (1 + mu) / 2.f;
	refreshR12();
	refreshStiffness();
}

ContinuumSegment& ContinuumSegment::operator=(ContinuumSegment& seg)
{
	this->E = seg.getE();
	this->mu = seg.getMu();
	this->d = seg.getD();
	this->L = seg.getL();
	this->rho = seg.getRho();
	this->delta_t = seg.getDelta_t();
	this->K = seg.getK();
	this->gamma = seg.getGamma();
	this->L0 = seg.getL0();
	this->G = E / (1 + mu) / 2.f;
	refreshR12();
	refreshStiffness();
	return *this;
}

void ContinuumSegment::refreshWholeStructure()
{
	refreshStiffness();
	refreshR12();
}

void ContinuumSegment::showAllParameters()
{
	std::cout << "E       : " << E << std::endl;
	std::cout << "mu      : " << mu << std::endl;
	std::cout << "d       : " << d << std::endl;
	std::cout << "L       : " << L << std::endl;
	std::cout << "rho     : " << rho << std::endl;
	std::cout << "delta_t : " << delta_t << std::endl;
	std::cout << "K       : " << K << std::endl;
	std::cout << "gamma   : " << gamma << std::endl;
	std::cout << "L0      : " << L0 << std::endl;
	std::cout << "G       : " << G << std::endl;
	std::cout << "r1       : " << r1.transpose() << std::endl;
	std::cout << "r2       : " << r2.transpose() << std::endl;
	std::cout << "Kb       : " << std::endl << Kb << std::endl;
	std::cout << "Ke       : " << std::endl << Ke << std::endl;

}

void ContinuumSegment::setE(float E)
{
	this->E = E;
	G = E / (1 + mu) / 2.f;
}

void ContinuumSegment::setMu(float mu)
{
	this->mu = mu;
	G = E / (1 + mu) / 2.f;
}

void ContinuumSegment::setD(float d)
{
	this->d = d;
}

void ContinuumSegment::setL(float L)
{
	this->L = L;
}

void ContinuumSegment::setL0(float L0)
{
	this->L0 = L0;
}

void ContinuumSegment::setRho(float rho)
{
	this->rho = rho;
}

void ContinuumSegment::setDeltaT(float delta_t)
{
	this->delta_t = delta_t;
}

void ContinuumSegment::setK(float K)
{
	this->K = K;
}

void ContinuumSegment::setGamma(float gamma)
{
	this->gamma = gamma;
}

void ContinuumSegment::refreshStiffness()
{
	if(d<0.5e-3f)
		A = keith_used::TSCR_PI * powf(d, 2);
	else
		A = keith_used::TSCR_PI * powf(d, 2) / 4;
	float I = keith_used::TSCR_PI * powf(d, 4) / 64;
	Ke = Eigen::Matrix3f::Identity();
	Ke(0, 0) = G * A;
	Ke(1, 1) = G * A;
	Ke(2, 2) = E * A;
	Kb = Eigen::Matrix3f::Identity();
	Kb(0, 0) = E * I;
	Kb(1, 1) = E * I;
	Kb(2, 2) = 2 * G * I;
}

void ContinuumSegment::refreshR12()
{
	r1(0) = cosf(delta_t) * rho;
	r1(1) = rho * sinf(delta_t);
	r1(2) = 0.f;
	r2(0) = cosf(delta_t + keith_used::TSCR_PI / 2.f) * rho;
	r2(1) = rho * sinf(delta_t + keith_used::TSCR_PI / 2.f);
	r2(2) = 0.f;
}

Eigen::Vector3f ContinuumSegment::getR1()
{
	return r1;
}

Eigen::Vector3f ContinuumSegment::getR2()
{
	return r2;
}

Eigen::Matrix3f ContinuumSegment::getKe()
{
	return Ke;
}

Eigen::Matrix3f ContinuumSegment::getKb()
{
	return Kb;
}

float ContinuumSegment::getL()
{
	return L;
}

float ContinuumSegment::getL0()
{
	return L0;
}

float ContinuumSegment::getA()
{
	return A;
}

float ContinuumSegment::getE()
{
	return E;
}

float ContinuumSegment::getK()
{
	return K;
}

float ContinuumSegment::getMu()
{
	return mu;
}

float ContinuumSegment::getD()
{
	return d;
}

float ContinuumSegment::getRho()
{
	return this->rho;
}

float ContinuumSegment::getDelta_t()
{
	return delta_t;
}

float ContinuumSegment::getGamma()
{
	return gamma;
}
