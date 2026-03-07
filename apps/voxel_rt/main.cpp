#include <print>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vrt/core/camera.hpp>
#include <vrt/core/buffer_2d.hpp>
#include <vrt/core/types.hpp>
#include <vrt/core/scene.hpp>

#include <vrt/math/ray.hpp>
#include <vrt/math/math.hpp>
#include <vrt/math/aabb.hpp>

#include <vrt/accel/blas.hpp>

#include <vrt/rt/hit.hpp>
#include <vrt/rt/intersector.hpp>

#include <vrt/gfx/window.hpp>
#include <vrt/gfx/presenter.hpp>


using namespace vrt;

void processInput(const Window& window, Camera& camera, float dt)
{
    if (window.is_key_pressed(GLFW_KEY_W))
        camera.process_keyboard(Direction::Forward, dt);
    if (window.is_key_pressed(GLFW_KEY_S))
        camera.process_keyboard(Direction::Backward, dt);
    if (window.is_key_pressed(GLFW_KEY_A))
        camera.process_keyboard(Direction::Left, dt);
    if (window.is_key_pressed(GLFW_KEY_D))
        camera.process_keyboard(Direction::Right, dt);
    if (window.is_key_pressed(GLFW_KEY_E))
        camera.process_keyboard(Direction::Up, dt);
    if (window.is_key_pressed(GLFW_KEY_Q))
        camera.process_keyboard(Direction::Down, dt);
}

int main()
{
    const int window_width = 1280;
    const int window_height = 720;

    const int resolution_width = 192;
    const int resolution_height = 108;

    Window window{ window_width, window_height, "voxel_rt" };
    Presenter presenter{ resolution_width, resolution_height };
    Buffer2D<glm::vec3> color_buffer{ resolution_width,resolution_height };

    Camera camera{
        static_cast<float>(resolution_width) / resolution_height,
        radians(120.f),
        -90.f, 0, glm::vec3{0},
    };

    
    Scene scene{};
    Intersector intersector{ scene };
    u8 depth{ 7 }; // 2**5 = 32
    glm::vec3 position{ 0.0f };
    u32 root_idx = scene.blas().build(position, depth);
    scene.add_instance({ root_idx, depth, glm::translate(glm::mat4(1.0f), position) });
    scene.print_debug();

    float current_frame_time = 0.0f;
    float last_frame_time = 0.0f;
    float delta_time = 0.0f;

    float start_frame = glfwGetTime();
    int frame_count = 0;

    while (!window.should_close())
    {
        current_frame_time = glfwGetTime();
        delta_time = current_frame_time - last_frame_time;
        last_frame_time = current_frame_time;
        
        window.pool_events();

        float dx = 0.0f, dy = 0.0f;
        window.get_mouse_delta(dx, dy);
        processInput(window, camera, delta_time);

        camera.process_mouse_movement(dx, dy);

        //#pragma omp parallel for schedule(dynamic, 1)
        for (int y = 0; y < resolution_height; ++y)
        {
            for (int x = 0; x < resolution_width; ++x)
            {
                float u = (float(x) / resolution_width) * 2 - 1;
                float v = 1.0f - (float(y) / resolution_height) * 2.0f;

                glm::vec3 d = normalize(camera.get_ray(u, v).direction);
                Ray r{ camera.position(), d, 1.0f / d };
                Hit result = { .t = INFINITY };
                for (const auto& instance : scene.instances()) 
                {
                    Hit temp = intersector.intersect(r, instance);
                    if (temp.t < result.t) {
                        result = temp;
                    }
                }

                float fade = std::clamp(1.0f - std::pow(result.t / 16, 0.25f), 0.0f, 1.0f);
                float lightness = std::clamp(dot(result.normal, normalize(glm::vec3{ 5,10,8 })), 0.f, 1.f);
                color_buffer(x, y) = glm::vec3{ fade + lightness };
            }
        }

        presenter.present(reinterpret_cast<const float*>(color_buffer.data()));
        window.swap_buffers();

        frame_count++;
        float time = (glfwGetTime() - start_frame) / frame_count;

        window.set_window_title("fps/ms: " + std::to_string(1.0f / delta_time) + "/" + std::to_string(delta_time * 1000) +
            "avg fps/ms: " + std::to_string(1.0f / time) + "/" + std::to_string(time  * 1000));
    }
}