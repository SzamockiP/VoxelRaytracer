#pragma once
#include <vrt/core/types.hpp>
#include <glm/glm.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	struct AABB
	{
		glm::vec3 min;
		glm::vec3 max;

		const Hit intersect(const Ray& ray) const noexcept;
	};
}