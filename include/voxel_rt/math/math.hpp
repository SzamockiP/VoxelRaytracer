#pragma once
#include "voxel_rt/math/vec3.hpp"
#include <numbers>


namespace vrt
{
	inline constexpr float pi_f = std::numbers::pi_v<float>;
	inline constexpr double pi_d = std::numbers::pi_v<double>;

	inline constexpr float deg_to_rad(float deg) noexcept
	{
		return pi_f / 180 * deg;
	}

	inline constexpr double deg_to_rad(double deg) noexcept
	{
		return pi_d / 180 * deg;
	}
}