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
    const float major_radius = 16.0f;
    const float minor_radius = 8.0f;

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

RayHit trace_ray(const DagPoolManager& manager, const Ray& ray, std::uint32_t current_index, int depth, const AABB& bounds)
{
    RayHit bounds_hit = bounds.intersect(ray);
    if (bounds_hit.t == float_inf || current_index == EMPTY)
    {
        return { float_inf };
    }
    Vec3 size = bounds.max - bounds.min;
    Vec3 half_size = size / 2;

    Vec3 right = { half_size.x, 0, 0 };
    Vec3 up    = { 0, half_size.y, 0 };
    Vec3 back  = { 0, 0, half_size.z };

    AABB child_bounds[8] = {
        { bounds.min, bounds.max - back - up - right },
        { bounds.min + right, bounds.max - back - up },
        { bounds.min + up, bounds.max - back - right },
        { bounds.min + right + up, bounds.max - back },
        { bounds.min + back, bounds.max - right - up },
        { bounds.min + back + right, bounds.max - up },
        { bounds.min + up + back, bounds.max - right },
        { bounds.min + back + right + up, bounds.max }
    };

    if (depth > 0)
    {
        const Node& n = manager.dagPool().nodes[current_index];
        ChildHit hits[8];
        int hit_count = 0;

        for (int i = 0; i < 8; i++)
        {
            if (n.indices[i] == EMPTY) continue;

            RayHit hit = child_bounds[i].intersect(ray);
            if (hit.t != float_inf)
            {
                hits[hit_count++] = { n.indices[i], hit.t, child_bounds[i] };
            }
        }

        
        std::sort(hits, hits + hit_count, [](const ChildHit& a, const ChildHit& b)
            {
                return a.t_min < b.t_min;
            });

        
        for (int i = 0; i < hit_count; ++i)
        {
            RayHit result = trace_ray(manager, ray, hits[i].index, depth - 1, hits[i].bounds);

            if (result.t != float_inf)
            {
                return result;
            }
        }
    }
    
    else
    {
        const Leaf& l = manager.dagPool().leaves[current_index];
        ChildHit hits[8];
        int hit_count = 0;

        for (int i = 0; i < 8; i++)
        {
            if (l.voxels[i] == 0) continue;

            RayHit hit = child_bounds[i].intersect(ray);
            if (hit.t != float_inf)
            {
                hits[hit_count++] = { l.voxels[i], hit.t, child_bounds[i]};
            }
        }

        if (hit_count > 0)
        {
            std::sort(hits, hits + hit_count, [](const ChildHit& a, const ChildHit& b)
                {
                    return a.t_min < b.t_min;
                });

            return { hits[0].t_min };
        }
    }

    return { float_inf };
}

int main()
{
    Window window{ 720, 480, "voxel_rt" };

    while (!window.should_close())
    {
        window.pool_events();
        window.swap_buffers();
    }


    /*DagPoolManager manager{};
	const int width = 720;
	const int height = 480;
	Buffer2D<Vec3f> vec_dir_buffer{ width,height };
	Buffer2D<Vec3f> color_buffer{ width,height };

	Camera camera{
		Vec3f{32,70,72},
		normalize(Vec3f{0,-0.5,-0.5}),
        static_cast<float>(width) / height,
		deg_to_rad(120.f)
	};

	const AABB chunk_volume{ .min = {-64}, .max = {64} };

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			float u = (static_cast<float>(x) / vec_dir_buffer.width()) * 2 - 1;
			float v = 1.0f - ((static_cast<float>(y) + 0.5f) / vec_dir_buffer.height()) * 2.0f;
			vec_dir_buffer(x, y) = normalize(camera.get_ray(u, v).direction);
		}
	}

	std::uint32_t root_idx = build_tree(manager, chunk_volume.min, chunk_volume.max);

    float highest_t = 0;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const Ray r{ camera.position(), vec_dir_buffer(x, y) , 1 / vec_dir_buffer(x, y) };
			RayHit result = trace_ray(manager, r, root_idx, 6, chunk_volume);
			color_buffer(x, y) = result.t != float_inf ? Vec3f{ result.t } : Vec3f{ float_inf };
            highest_t = result.t > highest_t && result.t != float_inf ? result.t : highest_t;
		}
	}

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            color_buffer(x, y) = Vec3f{ 1 } - (color_buffer(x, y) / highest_t);
        }
    }

	PpmWriter writer{};

	writer.write("out.ppm", color_buffer);

    return 0;*/
}