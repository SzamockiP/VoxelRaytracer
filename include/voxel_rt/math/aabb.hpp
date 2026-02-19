#pragma once
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/math/ray.hpp"

namespace vrt
{
	struct AABB
	{
		Vec3f min;
		Vec3f max;

		constexpr bool intersect(const Ray& ray) const noexcept;
	};
}