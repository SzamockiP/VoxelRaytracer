#include <vrt/core/camera.hpp>
#include <vrt/math/math.hpp>
#include <vrt/math/vec3.hpp>
#include <cmath>
#include <cassert>
#include <algorithm>

vrt::Camera::Camera(float aspect_ratio, float fov, float yaw,
	float pitch, Vec3f position, Vec3f world_up) :
	aspect_ratio_(aspect_ratio), fov_(fov), yaw_(yaw), pitch_(pitch), position_(position), world_up_(world_up)
{
	assert(fov > 0 || fov < pi_f);
	
	UpdateVectors();

	half_viewport_width_ = std::tanf(fov * 0.5f);
	half_viewport_height_ = half_viewport_width_ / aspect_ratio_;
};

vrt::Ray vrt::Camera::get_ray(float u, float v) const noexcept
{
	Vec3f dir_world = {
		normalize(
			right_ * (u * half_viewport_width_) +
			up_ * (v * half_viewport_height_) +
			front_
		)
	};

	return Ray{ position_, dir_world };

}

void vrt::Camera::UpdateVectors() noexcept
{
	float yaw_rad = radians(yaw_);
	float pitch_rad = radians(pitch_);

	Vec3f front;

	front.x = std::cos(yaw_rad) * std::cos(pitch_rad);
	front.y = std::sin(pitch_rad);
	front.z = std::sin(yaw_rad) * std::cos(pitch_rad);

	front_	= normalize(front);
	right_	= normalize(cross(front_, world_up_));
	up_		= normalize(cross(right_, front_));
}

void vrt::Camera::ProcessKeyboard(Direction direction, float deltaTime) noexcept
{
	float velocity = movement_speed_ * deltaTime;
	switch (direction)
	{
		case Direction::Forward:   position_ += front_	* velocity; break;
		case Direction::Backward:  position_ -= front_	* velocity; break;
		case Direction::Left:      position_ -= right_	* velocity; break;
		case Direction::Right:     position_ += right_	* velocity; break;
		case Direction::Up:        position_ += up_		* velocity; break;
		case Direction::Down:      position_ -= up_		* velocity; break;
	}
}

void vrt::Camera::ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch) noexcept
{
	xoffset *= mouse_sensitivity_;
	yoffset *= mouse_sensitivity_;

	yaw_ += xoffset;
	pitch_ += yoffset;

	if (constrainPitch)
	{
		pitch_ = std::clamp(pitch_, MIN_PITCH, MAX_PITCH);
	}

	UpdateVectors();
}