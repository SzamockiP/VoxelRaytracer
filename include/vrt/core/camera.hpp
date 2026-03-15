#pragma once
#include <vrt/core/types.hpp>
#include <glm/glm.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/core/direction.hpp>

namespace vrt
{

	class Camera
	{
	public:
		static constexpr float DEFAULT_YAW = -90.0f;
		static constexpr float DEFAULT_PITCH = 0.0f;
		static constexpr float DEFAULT_SPEED = 100.0f;
		static constexpr float DEFAULT_SENSITIVITY = 0.1f;
		static constexpr float DEFAULT_FOV = 45.0f;

		static constexpr float MIN_FOV = 1.0f;
		static constexpr float MAX_FOV = 179.0f;

		static constexpr float MIN_PITCH = -89.0f;
		static constexpr float MAX_PITCH = 89.0f;

		Camera(float aspect_ratio, float fov, float yaw = DEFAULT_YAW,
			float pitch = DEFAULT_PITCH, glm::vec3 position = glm::vec3{0}, glm::vec3 world_up = glm::vec3{ 0,1,0 });

		void process_keyboard(Direction direction, float deltaTime) noexcept;
		void process_mouse_movement(float xoffset, float yoffset, bool constrainPitch = true) noexcept;


		glm::vec3 position() const noexcept { return position_; }
		glm::vec3 front() const noexcept { return front_; }
		float aspect_ratio() const noexcept { return aspect_ratio_; }
		float fov() const noexcept { return fov_; }

		Ray get_ray(float u, float v) const noexcept;

	private:
		glm::vec3 position_;
		glm::vec3 right_;
		glm::vec3 up_;
		glm::vec3 front_;
		glm::vec3 world_up_;

		float yaw_;
		float pitch_;

		float aspect_ratio_;
		float fov_;
		float mouse_sensitivity_{ DEFAULT_SENSITIVITY };
		float movement_speed_{ DEFAULT_SPEED };

		float half_viewport_width_;
		float half_viewport_height_;

		void update_vectors() noexcept;
	};
}