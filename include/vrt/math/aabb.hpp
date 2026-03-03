#pragma once
#include <vrt/math/vec3.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	struct AABB
	{
		Vec3f min;
		Vec3f max;

		const Hit intersect(const Ray& ray) const noexcept;
	};
}