#include <vrt/core/camera.hpp>
#include <vrt/math/math.hpp>
#include <vrt/math/vec3.hpp>
#include <cmath>
#include <cassert>

vrt::Camera::Camera(Vec3f position, Vec3f forward, float aspect_ratio, float fov, Vec3f world_up) :
	position_(position), forward_(forward), aspect_ratio_(aspect_ratio), fov_(fov), world_up_(world_up)
{
	assert(fov > 0 || fov < pi_f);
	
	right_ = normalize(cross(forward, world_up));
	up_ = normalize(cross(right_, forward));

	half_viewport_width_ = std::tanf(fov * 0.5f);
	half_viewport_height_ = half_viewport_width_ / aspect_ratio_;
};

vrt::Ray vrt::Camera::get_ray(float u, float v) const noexcept
{
	Vec3f dir_world = {
		normalize(
			right_ * (u * half_viewport_width_) +
			up_ * (v * half_viewport_height_) +
			forward_
		)
	};

	return Ray{ position_, dir_world };

}