#include <print>
#include <cmath>
#include <algorithm>
#include <GLFW/glfw3.h>

#include <vrt/core/camera.hpp>
#include <vrt/core/buffer_2d.hpp>

#include <vrt/math/ray.hpp>
#include <vrt/math/vec3.hpp>
#include <vrt/math/math.hpp>
#include <vrt/math/aabb.hpp>

#include <vrt/accel/blas/blas_manager.hpp>

#include <vrt/rt/hit.hpp>
#include <vrt/rt/intersector.hpp>

#include <vrt/gfx/window.hpp>
#include <vrt/gfx/presenter.hpp>

using namespace vrt;

constexpr float float_inf = std::numeric_limits<float>::infinity();

std::uint32_t shape_gyroid(const Vec3f& pos)
{
    Vec3f p = pos * 0.5f;

    float val = std::sin(p.x) * std::cos(p.y) +
        std::sin(p.y) * std::cos(p.z) +
        std::sin(p.z) * std::cos(p.x);

    return (val > -0.3f && val < 0.3f) ? 1 : 0;
}
std::uint32_t shape_torus(const Vec3f& pos)
{
    const float major_radius = 8.0f;
    const float minor_radius = 4.0f;

    float q = std::sqrt(pos.x * pos.x + pos.z * pos.z) - major_radius;

    float distance = std::sqrt(q * q + pos.y * pos.y);

    return distance <= minor_radius ? 1 : 0;
}

static Voxel shape(const Vec3f& pos)
{
    return shape_gyroid(pos) == 1 ? Voxel::FULL : Voxel::EMPTY;
}

std::uint32_t build_tree(BlasManager& manager, Vec3i min, Vec3i max)
{
	Vec3i size = max - min;
	Vec3i half_size = size / 2;
	Vec3i center = min + half_size;

	if (max - min == Vec3{2})
	{
		Leaf l{ 
			shape(center + Vec3f{ -0.5f, -0.5f, -0.5f }),
			shape(center + Vec3f{  0.5f, -0.5f, -0.5f }),
			shape(center + Vec3f{ -0.5f,  0.5f, -0.5f }),
			shape(center + Vec3f{  0.5f,  0.5f, -0.5f }),
			shape(center + Vec3f{ -0.5f, -0.5f,  0.5f }),
			shape(center + Vec3f{  0.5f, -0.5f,  0.5f }),
			shape(center + Vec3f{ -0.5f,  0.5f,  0.5f }),
			shape(center + Vec3f{  0.5f,  0.5f,  0.5f })
		};

		return manager.AddLeaf(l);
	}
	else
	{
		Vec3i right = { half_size.x, 0, 0 };
		Vec3i up	   = { 0, half_size.y, 0 };
		Vec3i back  = { 0, 0, half_size.z };

		Node n = {
			build_tree(manager, min, max - right - up - back),
			build_tree(manager, min + right, max - back - up),
			build_tree(manager, min + up, max - back - right),
			build_tree(manager, min + right + up, max - back),
			build_tree(manager, min + back, max - right - up),
			build_tree(manager, min + back + right, max - up),
			build_tree(manager, min + up + back, max - right),
			build_tree(manager, min + up + back + right, max)
		};
		return manager.AddNode(n);
	}
}

void processInput(const Window& window, Camera& camera, float dt)
{
    if (window.is_key_pressed(GLFW_KEY_W))
        camera.ProcessKeyboard(Direction::Forward, dt);
    if (window.is_key_pressed(GLFW_KEY_S))
        camera.ProcessKeyboard(Direction::Backward, dt);
    if (window.is_key_pressed(GLFW_KEY_A))
        camera.ProcessKeyboard(Direction::Left, dt);
    if (window.is_key_pressed(GLFW_KEY_D))
        camera.ProcessKeyboard(Direction::Right, dt);
    if (window.is_key_pressed(GLFW_KEY_E))
        camera.ProcessKeyboard(Direction::Up, dt);
    if (window.is_key_pressed(GLFW_KEY_Q))
        camera.ProcessKeyboard(Direction::Down, dt);
}

int main()
{
    const int window_width = 1280;
    const int window_height = 720;

    const int resolution_width = 192;
    const int resolution_height = 144;

    Window window{ window_width, window_height, "voxel_rt" };
    Presenter presenter{ resolution_width, resolution_height };
    Buffer2D<Vec3f> color_buffer{ resolution_width,resolution_height };

    Camera camera{
        static_cast<float>(resolution_width) / resolution_height,
        radians(120.f),
        -90.f, 0, {0},
    };

    BlasManager manager{};
    const AABB chunk_volume{ .min = {-64}, .max = {64} };

    Intersector intersector{ manager };

    std::uint32_t root_idx = build_tree(manager, chunk_volume.min, chunk_volume.max);

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

        camera.ProcessMouseMovement(dx, dy);

        //#pragma omp parallel for schedule(dynamic, 1)
        for (int y = 0; y < resolution_height; ++y)
        {
            for (int x = 0; x < resolution_width; ++x)
            {
                float u = (float(x) / resolution_width) * 2 - 1;
                float v = 1.0f - (float(y) / resolution_height) * 2.0f;

                Vec3f d = normalize(camera.get_ray(u, v).direction);
                Ray r{ camera.position(), d, 1.0f / d };

                Hit result = intersector.intersect(r, root_idx, 6, { 0 });
                float fade = std::clamp(1.0f - std::pow(result.t / 16, 0.25f), 0.0f, 1.0f);
                float lightness = std::clamp(dot(result.normal, normalize(Vec3f{ 5,10,8 })), 0.f, 1.f);
                color_buffer(x, y) = {  fade + lightness};
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