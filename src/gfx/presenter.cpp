#include <vrt/gfx/presenter.hpp>
#include <glad/glad.h>


const std::string vertex_shader_source = R"(
        #version 330 core
        layout (location = 0) in vec2 aPos;
        out vec2 TexCoords;
        void main() {
            gl_Position = vec4(aPos.x, aPos.y, 0.0, 1.0);
            TexCoords = TexCoords = vec2(aPos.x * 0.5 + 0.5, 0.5 - aPos.y * 0.5);
        }
    )";

const std::string fragment_shader_source = R"(
        #version 330 core
        out vec4 FragColor;
        in vec2 TexCoords;
        uniform sampler2D screenTexture;
        void main() {
            FragColor = texture(screenTexture, TexCoords);
        }
    )";

vrt::Presenter::Presenter(int width, int height) :
	width_(width), height_(height)
{
    shader_program_ = create_shader_program(vertex_shader_source, fragment_shader_source);

    float vertices[] = {
        -1.0f,  3.0f,
        -1.0f, -1.0f,
         3.0f, -1.0f,
    };

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    
    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width_, height_, 0, GL_RGB, GL_FLOAT, nullptr);
}

vrt::Presenter::~Presenter()
{
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
    glDeleteProgram(shader_program_);
    glDeleteTextures(1, &texture_id_);
}

void vrt::Presenter::present(const float* rgb_data)
{
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGB, GL_FLOAT, rgb_data);

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shader_program_);
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

unsigned int vrt::Presenter::compile_shader(unsigned int type, const std::string& source)
{
    unsigned int id = glCreateShader(type);
    const char* src = source.c_str();

    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);

    return id;
}

unsigned int vrt::Presenter::create_shader_program(const std::string& vertex_src, const std::string& fragment_src)
{
    unsigned int program = glCreateProgram();
    unsigned int vs = compile_shader(GL_VERTEX_SHADER, vertex_src);
    unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, fragment_src);

    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}