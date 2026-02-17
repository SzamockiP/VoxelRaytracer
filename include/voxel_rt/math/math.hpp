#pragma once
#include "voxel_rt/math/vec3.hpp"
#include <numbers>


namespace vrt
{

inline constexpr float pi = std::numbers::pi_v<float>;

inline float lenght2(Vec3f v) noexcept
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline float lenght(Vec3f v) noexcept
{
	return std::sqrtf(lenght2(v));
}

inline float dot(Vec3f a, Vec3f b) noexcept
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3f cross(Vec3f a, Vec3f b) noexcept
{
	return Vec3f{
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	};
}

inline Vec3f normalize(Vec3f v) noexcept
{
	float len = lenght(v);
	return (len > 0.0f) ? Vec3f{ v.x / len, v.y / len, v.z / len } : Vec3f{};
}

}