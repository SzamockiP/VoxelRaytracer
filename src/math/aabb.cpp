#include <vrt/math/aabb.hpp>
#include <algorithm>

const vrt::Hit vrt::AABB::intersect(const Ray& ray) const noexcept
{
	float tmin = 0.0f;
	float tmax = std::numeric_limits<float>::infinity();

	for (int i = 0; i < 3; ++i)
	{
		float t1 = (min[i] - ray.origin[i]) * ray.direction_inverse[i];
		float t2 = (max[i] - ray.origin[i]) * ray.direction_inverse[i];
		tmin = glm::max(tmin, glm::min(t1, t2));
		tmax = glm::min(tmax, glm::max(t1, t2));
	}

	if (tmax >= tmin) 
		return { .t = tmin }; 
	else 
		return { .t = std::numeric_limits<float>::infinity() };
}