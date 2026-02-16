#pragma once
#include "voxel_rt/math/vec3.hpp"
#include <numbers>


namespace vrt
{

inline constexpr float pi = std::numbers::pi_v<float>;

inline float lenght2(Vec3 v) noexcept
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline float lenght(Vec3 v) noexcept
{
	return std::sqrtf(lenght2(v));
}

inline float dot(Vec3 a, Vec3 b) noexcept
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 cross(Vec3 a, Vec3 b) noexcept
{
	return Vec3{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

inline Vec3 normalize(Vec3 v) noexcept
{
	float len = lenght(v);
	return (len > 0.0f) ? Vec3{ v.x / len, v.y / len, v.z / len } : Vec3{};
}

}