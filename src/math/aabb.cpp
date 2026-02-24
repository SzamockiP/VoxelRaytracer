#include <vrt/math/aabb.hpp>
#include <algorithm>

vrt::RayHit vrt::AABB::intersect(const Ray& ray) const noexcept
{
	float tmin = 0.0f;
	float tmax = std::numeric_limits<float>::infinity();

	for (int i = 0; i < 3; ++i)
	{
		float t1 = (min[i] - ray.origin[i]) * ray.direction_inverse[i];
		float t2 = (max[i] - ray.origin[i]) * ray.direction_inverse[i];

		tmin = std::max(tmin, std::min(t1, t2));
		tmax = std::min(tmax, std::max(t1, t2));
	}

	if (tmax >= tmin) 
		return { .t = tmin }; 
	else 
		return { .t = std::numeric_limits<float>::infinity() };
}