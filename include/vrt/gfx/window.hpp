#pragma once
#include <cstdint>
#include <string>

struct GLFWwindow;

namespace vrt
{
	class Window
	{
	public:
		Window(int width, int height, const std::string& title);

		~Window();	

		Window(const Window&) = delete;
		Window& operator=(const Window&) = delete;

		bool should_close() const;
		void pool_events();
		void swap_buffers();

		bool is_key_pressed(int key) const;
		void get_mouse_delta(float& out_dx, float& out_dy);

		int get_width() const noexcept { return width_; }
		int get_height() const noexcept { return height_; }

		void set_window_title(const std::string& title) noexcept;

	private:
		GLFWwindow* window_{ nullptr };
		int width_;
		int height_;

		struct MouseState
		{
			float dx = 0.0f;
			float dy = 0.0f;
			float last_x = 0.0f;
			float last_y = 0.0f;
			bool first_mouse = true;
		} mouse_;

		static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	};
}