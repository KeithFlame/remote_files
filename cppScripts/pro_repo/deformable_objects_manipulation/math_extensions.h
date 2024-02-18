#pragma once


namespace Keith {
	constexpr double PI = 3.1416;
	constexpr double PI_2 = PI / 2;
	constexpr double PI_4 = PI / 4;
	template<typename T>
	T deg2rad(T deg)
	{
		return deg * PI / 180f;
	}

	template <typename T>
	T rad2deg(T rad)
	{
		return rad * 180f / PI;
	}
}
