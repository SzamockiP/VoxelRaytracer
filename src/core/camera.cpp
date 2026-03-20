#include <vrt/core/camera.hpp>
#include <glm/glm.hpp>
#include <algorithm>

vrt::Camera::Camera(
	float aspect_ratio,
	float fov,
	float yaw,
	float pitch,
	float speed,
	glm::vec3 position,
	glm::vec3 world_up
) :
	aspect_ratio_(aspect_ratio), fov_(fov), yaw_(yaw), pitch_(pitch), movement_speed_(speed), position_(position), world_up_(world_up)
{
	update_vectors();

	half_viewport_width_ = glm::tan(fov * 0.5f);
	half_viewport_height_ = half_viewport_width_ / aspect_ratio_;
};

vrt::Ray vrt::Camera::get_ray(float u, float v) const noexcept
{
	glm::vec3 dir_world = {
		normalize(
			right_ * (u * half_viewport_width_) +
			up_ * (v * half_viewport_height_) +
			front_
		)
	};

	return Ray{ position_, dir_world };

}

void vrt::Camera::update_vectors() noexcept
{
	float yaw_rad = glm::radians(yaw_);
	float pitch_rad = glm::radians(pitch_);

	glm::vec3 front;

	front.x = glm::cos(yaw_rad) * glm::cos(pitch_rad);
	front.y = glm::sin(pitch_rad);
	front.z = glm::sin(yaw_rad) * glm::cos(pitch_rad);

	front_	= normalize(front);
	right_	= normalize(cross(front_, world_up_));
	up_		= normalize(cross(right_, front_));
}

void vrt::Camera::process_keyboard(Direction direction, float deltaTime) noexcept
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

void vrt::Camera::process_mouse_movement(float xoffset, float yoffset, bool constrainPitch) noexcept
{
	xoffset *= mouse_sensitivity_;
	yoffset *= mouse_sensitivity_;

	yaw_ += xoffset;
	pitch_ += yoffset;

	if (constrainPitch)
	{
		pitch_ = std::clamp(pitch_, MIN_PITCH, MAX_PITCH);
	}

	update_vectors();
}