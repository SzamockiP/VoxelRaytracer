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

static Voxel shape(const Vec3f& pos)
{
    return shape_gyroid(pos) == 1 ? Voxel::FULL : Voxel::EMPTY;
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

struct StackFrame
{
    std::uint32_t node_index;
    float tx, ty, tz;
    float t_exit;
    std::uint8_t oct_idx;
};

RayHit trace_ray(const DagPoolManager& manager,
    const Ray& ray,
    std::uint32_t root_index,
    int max_depth,
    Vec3f root_center)
{
    constexpr float INF = std::numeric_limits<float>::infinity();

    // mirror ray
    Vec3f O = ray.origin;
    Vec3f invD = ray.direction_inverse;
    Vec3f C = root_center;
    std::uint8_t a = 0; // axis inversion mask

    if (invD.x < 0.0f) { O.x = -O.x; invD.x = -invD.x; C.x = -C.x; a |= 1; }
    if (invD.y < 0.0f) { O.y = -O.y; invD.y = -invD.y; C.y = -C.y; a |= 2; }
    if (invD.z < 0.0f) { O.z = -O.z; invD.z = -invD.z; C.z = -C.z; a |= 4; }

    float t_enter = 0.0f;
    float t_exit = INF;

    const float root_half = float(1u << max_depth);

    // Root slab test
    float t0x = (C.x - root_half - O.x) * invD.x;
    float t1x = (C.x + root_half - O.x) * invD.x;
    t_enter = std::max(t_enter, t0x);
    t_exit = std::min(t_exit, t1x);

    float t0y = (C.y - root_half - O.y) * invD.y;
    float t1y = (C.y + root_half - O.y) * invD.y;
    t_enter = std::max(t_enter, t0y);
    t_exit = std::min(t_exit, t1y);

    float t0z = (C.z - root_half - O.z) * invD.z;
    float t1z = (C.z + root_half - O.z) * invD.z;
    t_enter = std::max(t_enter, t0z);
    t_exit = std::min(t_exit, t1z);

    t_enter = std::max(t_enter, 0.0f);

    if (t_exit < t_enter)
        return { .t = INF, .normal = {0}, .voxel = Voxel::EMPTY };

    // Track the axis of the last ADVANCE step that set t_enter (0=X,1=Y,2=Z)
    // Initialize from root slab entry (argmax of t0x/t0y/t0z).
    std::uint8_t last_axis = 0;
    {
        float best = t0x;
        last_axis = 0;
        if (t0y > best) { best = t0y; last_axis = 1; }
        if (t0z > best) { /*best = t0z;*/ last_axis = 2; }
        // If we clamped t_enter to 0, last_axis is only a fallback; it will be overwritten on first ADVANCE.
    }

    StackFrame stack[32];
    int sp = 0;

    int depth = max_depth;
    std::uint32_t current_index = root_index;

    float half = root_half;

    // center axis (mid-planes of current node)
    float txm = (C.x - O.x) * invD.x;
    float tym = (C.y - O.y) * invD.y;
    float tzm = (C.z - O.z) * invD.z;

    std::uint8_t oct_idx =
        (std::uint8_t(txm <= t_enter) << 0) |
        (std::uint8_t(tym <= t_enter) << 1) |
        (std::uint8_t(tzm <= t_enter) << 2);

    float tx = (txm > t_enter) ? txm : INF;
    float ty = (tym > t_enter) ? tym : INF;
    float tz = (tzm > t_enter) ? tzm : INF;

    while (true)
    {
        // 1. PROCESS / DESCEND
        if (depth > 0)
        {
            const std::uint32_t child_index =
                manager.dagPool().nodes[current_index].indices[oct_idx ^ a];

            if (child_index != EMPTY)
            {
                // t_next = min(tx,ty,tz)
                float t_next = tx;
                if (ty < t_next) t_next = ty;
                if (tz < t_next) t_next = tz;

                const float child_t_exit = (t_exit < t_next) ? t_exit : t_next;

                // push frame
                stack[sp++] = { current_index, tx, ty, tz, t_exit, oct_idx };

                // descent
                half *= 0.5f;
                const float offset = half;

                txm += ((oct_idx & 1) ? +offset : -offset) * invD.x;
                tym += ((oct_idx & 2) ? +offset : -offset) * invD.y;
                tzm += ((oct_idx & 4) ? +offset : -offset) * invD.z;

                current_index = child_index;
                t_exit = child_t_exit;
                --depth;

                oct_idx =
                    (std::uint8_t(txm <= t_enter) << 0) |
                    (std::uint8_t(tym <= t_enter) << 1) |
                    (std::uint8_t(tzm <= t_enter) << 2);

                tx = (txm > t_enter) ? txm : INF;
                ty = (tym > t_enter) ? tym : INF;
                tz = (tzm > t_enter) ? tzm : INF;

                continue;
            }
        }
        else
        {
            // leaf
            Voxel v = manager.dagPool().leaves[current_index].voxels[oct_idx ^ a];
            if (v != Voxel::EMPTY)
            {
                // Face normal from last ADVANCE axis (cheap and correct for your stepping)
                Vec3f n{ 0.f, 0.f, 0.f };

                // In mirrored space, direction is always +, so we always enter through the "negative" face.
                if (last_axis == 0) n.x = -1.f;
                else if (last_axis == 1) n.y = -1.f;
                else                     n.z = -1.f;

                // un-mirror back to world space
                if (a & 1) n.x = -n.x;
                if (a & 2) n.y = -n.y;
                if (a & 4) n.z = -n.z;

                return { .t = t_enter, .normal = n, .voxel = v };
            }
        }

        // 3. ADVANCE / POP
        while (true)
        {
            int axis = 0;
            float t_next = tx;
            if (ty < t_next) { t_next = ty; axis = 1; }
            if (tz < t_next) { t_next = tz; axis = 2; }

            // POP
            if (t_next > t_exit)
            {
                if (sp == 0)
                    return { .t = INF, .normal = {0}, .voxel = Voxel::EMPTY };

                // ascend
                ++depth;

                const float offset = half;

                // pop frame
                const StackFrame& frame = stack[--sp];

                current_index = frame.node_index;
                tx = frame.tx;
                ty = frame.ty;
                tz = frame.tz;
                t_exit = frame.t_exit;
                oct_idx = frame.oct_idx;

                txm -= ((oct_idx & 1) ? +offset : -offset) * invD.x;
                tym -= ((oct_idx & 2) ? +offset : -offset) * invD.y;
                tzm -= ((oct_idx & 4) ? +offset : -offset) * invD.z;

                half *= 2.0f;

                continue;
            }

            // ADVANCE to next boundary
            oct_idx |= std::uint8_t(1u << axis);
            if (axis == 0) tx = INF;
            else if (axis == 1) ty = INF;
            else                tz = INF;

            t_enter = t_next;
            last_axis = static_cast<std::uint8_t>(axis);
            break;
        }
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

        //#pragma omp parallel for schedule(dynamic, 1)
        for (int y = 0; y < resolution_height; ++y)
        {
            for (int x = 0; x < resolution_width; ++x)
            {
                float u = (float(x) / resolution_width) * 2 - 1;
                float v = 1.0f - (float(y) / resolution_height) * 2.0f;

                Vec3f d = normalize(camera.get_ray(u, v).direction);
                Ray r{ camera.position(), d, 1.0f / d };

                RayHit result = trace_ray(manager, r, root_idx, 6, { 0 });
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