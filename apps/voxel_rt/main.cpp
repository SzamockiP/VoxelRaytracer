#include <print>
#include <vrt/core/camera.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/math/vec3.hpp>
#include <vrt/math/math.hpp>
#include <vrt/io/ppm_writer.hpp>
#include <vrt/core/buffer_2d.hpp>
#include <vrt/math/aabb.hpp>
#include <vrt/voxel/dag_pool_manager.hpp>
#include <cassert>
#include <vrt/core/ray_hit.hpp>
#include <algorithm>
#include <cmath>
#include <vrt/gfx/window.hpp>
#include <vrt/gfx/presenter.hpp>
#include <GLFW/glfw3.h>

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

static std::uint32_t shape(const Vec3f& pos)
{
    return shape_gyroid(pos);
}

std::uint32_t build_tree(DagPoolManager& manager, Vec3i min, Vec3i max)
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

struct ChildHit
{
    std::uint32_t index{};
    float t_min{};
	AABB bounds;
};

RayHit trace_ray(const DagPoolManager& manager, const Ray& ray, std::uint32_t current_index, int depth, Vec3f center)
{
    // parent bounds test
    float t_enter = 0.0f;
    float t_exit = INFINITY;
    float half = 1 << depth;

    for (int i = 0; i < 3; ++i)
    {
        float t1 = (center[i] - half - ray.origin[i]) * ray.direction_inverse[i];
        float t2 = (center[i] + half - ray.origin[i]) * ray.direction_inverse[i];

        t_enter = std::max(t_enter, std::min(t1, t2));
        t_exit = std::min(t_exit, std::max(t1, t2));
    }

    // miss?
    if(t_exit < std::max(t_enter, 0.0f))
        return { .t = INFINITY };
    

    Vec3 p = ray.origin + ray.direction * t_enter;

    bool x_bit = p.x >= center.x;
    bool y_bit = p.y >= center.y;
    bool z_bit = p.z >= center.z;

    std::uint8_t oct_idx = x_bit | (y_bit << 1) | (z_bit << 2);

    Vec3f t_plane = (center - ray.origin) * ray.direction_inverse;

    float txm = t_plane.x > t_enter ? t_plane.x : INFINITY;
    float tym = t_plane.y > t_enter ? t_plane.y : INFINITY;
    float tzm = t_plane.z > t_enter ? t_plane.z : INFINITY;

    for (int i = 0; i < 4; i++)
    {
        if (depth > 0)
        {
            if (manager.dagPool().nodes[current_index].indices[oct_idx] != EMPTY)
            {
                float o = half * 0.5f;
                Vec3f offset = {
                    x_bit ? o : -o,
                    y_bit ? o : -o,
                    z_bit ? o : -o,
                };

                RayHit result = trace_ray(
                    manager,
                    ray,
                    manager.dagPool().nodes[current_index].indices[oct_idx],
                    depth - 1,
                    center + offset
                );

                if (result.t != INFINITY)
                {
                    return result;
                }
            }
        }
        else
        {
            if (manager.dagPool().leaves[current_index].voxels[oct_idx] != 0)
            {
                return { .t = t_enter };
            }
        }

        float t_next = std::min({ txm, tym, tzm });

        if (t_next > t_exit || t_next == INFINITY)
            break;

        if      (t_next == txm) { oct_idx ^= 1; x_bit = !x_bit; txm = INFINITY; }
        else if (t_next == tym) { oct_idx ^= 2; y_bit = !y_bit; tym = INFINITY; }
        else                    { oct_idx ^= 4; z_bit = !z_bit; tzm = INFINITY; }

        t_enter = t_next;
    }

    return { .t = INFINITY };
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
        -90.f, 0, {64},
    };

    DagPoolManager manager{};
    const AABB chunk_volume{ .min = {-64}, .max = {64} };

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

        #pragma omp parallel for schedule(dynamic, 1)
        for (int y = 0; y < resolution_height; ++y)
        {
            for (int x = 0; x < resolution_width; ++x)
            {
                float u = (float(x) / resolution_width) * 2 - 1;
                float v = 1.0f - (float(y) / resolution_height) * 2.0f;

                Vec3f d = normalize(camera.get_ray(u, v).direction);
                Ray r{ camera.position(), d, 1.0f / d };

                RayHit result = trace_ray(manager, r, root_idx, 6, { 0 });
                color_buffer(x, y) = Vec3{ 1.0f - std::pow(result.t / 64, 0.25f) };
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