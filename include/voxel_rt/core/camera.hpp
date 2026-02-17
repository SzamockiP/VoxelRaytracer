#pragma once
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/core/ray.hpp"

namespace vrt
{

class Camera
{
public:
	Camera(Vec3f position, Vec3f forward, float aspect_ratio, float fov, Vec3f world_up = Vec3f{ 0,1,0 });

	Vec3f position() const noexcept { return position_; }
	Vec3f direction() const noexcept {	return forward_; }
	float aspect_ratio() const noexcept	{ return aspect_ratio_; }
	float fov() const noexcept { return fov_; }

	Ray get_ray(float u, float v) const noexcept;

private:
	Vec3f position_;
	Vec3f forward_;
	Vec3f world_up_;
	Vec3f right_;
	Vec3f up_;

	float aspect_ratio_;
	float fov_;
	float half_viewport_width_;
	float half_viewport_height_;

};

}