#pragma once
#include <string>

namespace vrt
{
	class Presenter
	{
	public:
		Presenter(int width, int height);
		~Presenter();

		Presenter(const Presenter&) = delete;
		Presenter& operator=(const Presenter&) = delete;

		void present(const float* rgb_data);

	private:
		unsigned int vao_, vbo_;
		unsigned int shader_program_;
		unsigned int texture_id_;
		int width_, height_;

		unsigned int compile_shader(unsigned int type, const std::string& source);
		unsigned int create_shader_program(const std::string& vertex_src, const std::string& fragment_src);
	};
}