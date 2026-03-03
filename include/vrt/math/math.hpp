#pragma once
#include <vrt/math/vec3.hpp>
#include <numbers>


namespace vrt
{
	inline constexpr float pi_f = std::numbers::pi_v<float>;
	inline constexpr double pi_d = std::numbers::pi_v<double>;

	inline constexpr float radians(float deg) noexcept
	{
		return pi_f / 180 * deg;
	}

	inline constexpr double degrees(double deg) noexcept
	{
		return pi_d / 180 * deg;
	}
}