#include <print>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

#include <vrt/core/camera.hpp>
#include <vrt/core/buffer_2d.hpp>
#include <vrt/core/types.hpp>

#include <vrt/math/ray.hpp>

#include <vrt/gfx/window.hpp>
#include <vrt/gfx/presenter.hpp>

#include <vrt/accel/dag_v1.hpp>
#include <vrt/accel/dag_v2.hpp>

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

    const int resolution_width = 320;
    const int resolution_height = 180;

    Window window{ window_width, window_height, "voxel_rt" };
    Presenter presenter{ resolution_width, resolution_height };
    Buffer2D<glm::vec3> color_buffer{ resolution_width,resolution_height };

    

    /*vrt::v1::Dag dag;
    int tree_depth = 10;
    auto root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/1024c.bin");*/
    //dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/1024c.vdag");

    vrt::v2::Dag dag;
    int tree_depth = 14;
    auto root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/8192c.bin");
    //dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/1024c.vdag");

    /*tree_depth = 11;
    root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/2048c.bin");
    dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/2048c.vdag");

    tree_depth = 12;
    root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/4096c.bin");
    dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/4096c.vdag");

    tree_depth = 13;
    root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/8192c.bin");
    dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/8192c.vdag");

    tree_depth = 14;
    root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/16384c.bin");
    dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/16384c.vdag");

    tree_depth = 15;
    root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/San_Miguel/bin/32768c.bin");
    dag.save("C:/Users/Piotr/Downloads/San_Miguel/vdag/32768c.vdag");*/

    // Pozycja startowa i speed kamery = 2^(depth-4), czyli ~1/16 rozmiaru sceny
    const float cam_scale = static_cast<float>(1u << std::max(0, tree_depth - 4));
    Camera camera{
        static_cast<float>(resolution_width) / resolution_height,
        glm::radians(120.f),
        -90.f,
        0,
        cam_scale,
        glm::vec3{ cam_scale }
    };

    //auto root = dag.build(tree_depth, "C:/Users/Piotr/Downloads/hairball/bin/4096.bin");
    //dag.save("C:/Users/Piotr/Downloads/hairball/vdag/4096.vdag");

    //auto root = dag.load("C:/Users/Piotr/Downloads/San_Miguel/vdag/2048.vdag");

    /*if (root.descriptor == 0)
    {
        std::cerr << "Blad: DAG jest pusty!\n";
        return -1;
    }*/

    //dag.debug();
    dag.print_stats();

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

#pragma omp parallel for schedule(dynamic, 1)
        for (int y = 0; y < resolution_height; ++y)
        {
            for (int x = 0; x < resolution_width; ++x)
            {
                // Mapowanie piksela na NDC [-1, 1] (v odwrócone: góra ekranu = +1)
                float u = (float(x) / resolution_width) * 2 - 1;
                float v = 1.0f - (float(y) / resolution_height) * 2.0f;

                glm::vec3 d = normalize(camera.get_ray(u, v).direction);
                Ray r{ camera.position(), d, 1.0f / d };
                v2::Dag::Hit result = dag.intersect(r, tree_depth, root);
                //v1::Dag::Hit result = dag.intersect(r, tree_depth, root);

                if (result.t == std::numeric_limits<float>::infinity())
                {
                    color_buffer(x, y) = glm::vec3{ 0.1f, 0.1f, 0.15f };
                }
                else
                {
                    glm::vec3 base_color{
                        result.voxel.r / 255.0f,
                        result.voxel.g / 255.0f,
                        result.voxel.b / 255.0f
                    };

                    glm::vec3 light_dir = glm::normalize(glm::vec3(0.5f, 1.0f, 0.3f));
                    float diff = glm::max(0.2f, glm::dot(result.normal, light_dir));

                    color_buffer(x, y) = base_color * diff;
                }
            }
        }

        presenter.present(reinterpret_cast<const float*>(color_buffer.data()));
        window.swap_buffers();

        frame_count++;
        float time = (glfwGetTime() - start_frame) / frame_count;

        window.set_window_title(
            "fps/ms: " + std::to_string(1.0f / delta_time) + "/" + std::to_string(delta_time * 1000) + "  |  " +
            "avg fps/ms: " + std::to_string(1.0f / time) + "/" + std::to_string(time * 1000));
    }
}