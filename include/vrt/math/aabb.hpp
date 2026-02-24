#pragma once
#include <vrt/math/vec3.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/core/ray_hit.hpp>

namespace vrt
{
	struct AABB
	{
		Vec3f min;
		Vec3f max;

		RayHit intersect(const Ray& ray) const noexcept;
	};
}