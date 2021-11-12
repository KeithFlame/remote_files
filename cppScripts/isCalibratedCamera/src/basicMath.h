#ifndef _BASIC_MATH_H_
#define _BASIC_MATH_H_
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
namespace math
{
	template<typename T1 = float, typename T2 = double>
	inline T1 getSTD(std::vector<T1> vec)
		{
			std::vector<T1> resultSet = vec;
			T1 sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
			T1 mean = sum / resultSet.size(); //��ֵ
			T1 accum = 0.0;
			std::for_each(std::begin(resultSet), std::end(resultSet), [&](const T1 d) {
				accum += (d - mean) * (d - mean);
				});

			T1 stdev = sqrt(accum / (resultSet.size() - 1)); //����
			return stdev;
		}


	template<typename T1 = float, typename T2 = float>
	inline T1 getMaxFloat(std::vector<T1> vec)
		{
			T1 res = -10000.f;
			for (int i = 0; i < vec.size(); i++)
				if (res < vec[i])
					res = vec[i];
			return res;
		}


	template<typename T1 = float, typename T2 = float>
	inline T1 getMinFloat(std::vector<T1> vec)
	{
		T1 res = 10000000.f;
		for (int i = 0; i < vec.size(); i++)
			if (res > vec[i])
				res = vec[i];
		return res;
	}

}

#endif //_BASIC_MATH_H_