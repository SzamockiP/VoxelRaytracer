#include <vrt/math/aabb.hpp>
#include <algorithm>
#include <glm/gtx/component_wise.hpp>

bool vrt::AABB::intersect(const Ray& ray) const noexcept
{
    float tmin = 0.0f;
    float tmax = std::numeric_limits<float>::infinity();

    glm::vec3 t1 = (min - ray.origin) * ray.direction_inverse;
    glm::vec3 t2 = (max - ray.origin) * ray.direction_inverse;

    glm::vec3 t_near = glm::min(t1, t2);
    glm::vec3 t_far = glm::max(t1, t2);

    tmin = glm::max(tmin, glm::compMax(t_near));
    tmax = glm::min(tmax, glm::compMin(t_far));

	return tmax >= tmin;
}