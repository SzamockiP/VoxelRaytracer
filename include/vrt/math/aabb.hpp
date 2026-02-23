#pragma once
#include <vrt/math/vec3.hpp>
#include <vrt/math/ray.hpp>

namespace vrt
{
	struct AABB
	{
		Vec3f min;
		Vec3f max;

		bool intersect(const Ray& ray) const noexcept;
	};
}