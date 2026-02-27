#include <vrt/gfx/window.hpp>
#include <glad/glad.h>
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

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		glfwTerminate();
		throw std::runtime_error("[GLAD] Could not initialize glad");
	}

	glfwSetWindowUserPointer(window_, this);

	glfwSetCursorPosCallback(window_, mouse_callback);

	glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
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

bool vrt::Window::is_key_pressed(int key) const
{
	return glfwGetKey(window_, key) == GLFW_PRESS;
}

void vrt::Window::get_mouse_delta(float& out_dx, float& out_dy)
{
	out_dx = mouse_.dx;
	out_dy = mouse_.dy;
	mouse_.dx = 0.0f;
	mouse_.dy = 0.0f;
}

void vrt::Window::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	Window* win = static_cast<Window*>(glfwGetWindowUserPointer(window));

	if (win->mouse_.first_mouse)
	{
		win->mouse_.last_x = static_cast<float>(xpos);
		win->mouse_.last_y = static_cast<float>(ypos);
		win->mouse_.first_mouse = false;
	}

	win->mouse_.dx += static_cast<float>(xpos) - win->mouse_.last_x;
	win->mouse_.dy += win->mouse_.last_y - static_cast<float>(ypos);

	win->mouse_.last_x = static_cast<float>(xpos);
	win->mouse_.last_y = static_cast<float>(ypos);
}

void vrt::Window::set_window_title(const std::string& title) noexcept
{
	glfwSetWindowTitle(window_, title.c_str());
}