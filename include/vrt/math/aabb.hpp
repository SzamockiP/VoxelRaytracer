#pragma once
#include <vrt/core/types.hpp>
#include <glm/glm.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	struct AABB
	{
		glm::vec3 min = glm::vec3(std::numeric_limits<float>::max());
		glm::vec3 max = glm::vec3(std::numeric_limits<float>::lowest());

		bool intersect(const Ray& ray) const noexcept;

		void grow(const glm::vec3& point) noexcept {
			min = glm::min(min, point);
			max = glm::max(max, point);
		}

		void grow(const AABB& other) noexcept{
			min = glm::min(min, other.min);
			max = glm::max(max, other.max);
		}

		glm::vec3 centroid() const noexcept{
			return (min + max) * 0.5f;
		}
	};
}