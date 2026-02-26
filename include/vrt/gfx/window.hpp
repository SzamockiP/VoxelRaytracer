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

		int get_width() const noexcept { return width_; }
		int get_height() const noexcept { return height_; }

	private:
		GLFWwindow* window_{ nullptr };
		int width_;
		int height_;

	};
}