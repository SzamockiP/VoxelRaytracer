#pragma once
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/core/ray.hpp"

namespace vrt
{

class Camera
{
public:
	Camera(Vec3 position, Vec3 forward, float aspect_ratio, float fov, Vec3 world_up);

	Vec3 position() const noexcept { return position_; }
	Vec3 direction() const noexcept {	return forward_; }
	float aspect_ratio() const noexcept	{ return aspect_ratio_; }
	float fov() const noexcept { return fov_; }

	Ray get_ray(float u, float v) const noexcept;

private:
	Vec3 position_;
	Vec3 forward_;
	Vec3 world_up_;
	Vec3 right_;
	Vec3 up_;

	float aspect_ratio_;
	float fov_;
	float half_viewport_width_;
	float half_viewport_height_;

};

}