#include <vrt/gfx/window.hpp>
#include <GLFW/glfw3.h>
#include <stdexcept>

vrt::Window::Window(int width, int height, const std::string& title):
	width_(width), height_(height)
{
	if (!glfwInit())
	{
		throw std::runtime_error("[GLFW] Could not initialize glfw");
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window_ = glfwCreateWindow(width_, height_, title.c_str(), nullptr, nullptr);

	if (!window_)
	{
		glfwTerminate();
		throw std::runtime_error("[GLFW] Could not create window");
	}

	glfwMakeContextCurrent(window_);
}



vrt::Window::~Window()
{
	if (window_)
	{
		glfwDestroyWindow(window_);
	}
	glfwTerminate();
}


bool vrt::Window::should_close() const
{
	return glfwWindowShouldClose(window_);
}

void vrt::Window::pool_events()
{
	glfwPollEvents();
}

void vrt::Window::swap_buffers()
{
	glfwSwapBuffers(window_);
}
