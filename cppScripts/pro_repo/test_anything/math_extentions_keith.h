#pragma once
#ifndef MATH_EXTENSIONS_KEITH_H_
#define MATH_EXTENSIONS_KEITH_H_
#include <Eigen/SVD>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

namespace keith_used {
	constexpr auto TSCR_PI = 3.141592654f;

	void pinv(Eigen::MatrixXf& outMatrix, const Eigen::MatrixXf inMatrix);
	void pinv(Eigen::MatrixXd& outMatrix, const Eigen::MatrixXd& inMatrix);
	std::vector<std::string> split(const std::string&, const std::string&);
	Eigen::MatrixXd readData(std::string);

	struct   TimerAvrg {
		std::vector<double> times;
		size_t curr = 0, n;
		std::chrono::high_resolution_clock::time_point begin, end;
		TimerAvrg(int _n = 30);
		inline void start() { begin = std::chrono::high_resolution_clock::now(); }
		inline void stop() {
			end = std::chrono::high_resolution_clock::now();
			double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
			if (times.size() < n) times.push_back(duration); else { times[curr] = duration; curr++; if (curr >= times.size()) curr = 0; }
		}
		double getAvrg();
	};


	inline Eigen::Matrix3d normalizeRotation(Eigen::Matrix3d rot)
	{
		rot.col(0) = rot.col(0) / rot.col(0).norm(); rot.col(1) = rot.col(1) / rot.col(1).norm();
		rot.col(2) = rot.col(0).cross(rot.col(1));
		rot.row(0) = rot.row(0) / rot.row(0).norm(); rot.row(1) = rot.row(1) / rot.row(1).norm();
		rot.row(2) = rot.row(0).cross(rot.row(1));
		return rot;
	}



	template<class T>
	T deg2rad(T t1)
	{
		return t1 * (T)TSCR_PI / (T)180;
	}

	template<class T>
	T rad2deg(T t1)
	{
		return t1 / (T)TSCR_PI * (T)180;
	}
}

#endif // !MATH_EXTENSIONS_KEITH_H_

