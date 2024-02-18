#include <iostream>
#include "timer.h"

int main()
{
	float xf = 1.235423524f;
	double xd = 1.235423524;
	auto tic = mmath::timer::getCurrentTimePoint();
	for (size_t i = 0; i < 1000000; i++)
	{
		xf *= xf; xf /= xf;
	}
	float t1 = mmath::timer::getDurationSince(tic);
	tic = mmath::timer::getCurrentTimePoint();
	for (size_t i = 0; i < 1000000; i++)
	{
		xd *= xd; xd /= xd;
	}
	float t2 = mmath::timer::getDurationSince(tic);
	std::cout << "duration: " << t1 << "\t" << t2 << std::endl;
}