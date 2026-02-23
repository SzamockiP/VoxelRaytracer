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
/*
using namespace vrt;
int main()
{
	const int width = 720;
	const int height = 480;
	Buffer2D<Vec3f> vec_dir_buffer{ width,height };
	Buffer2D<Vec3f> color_buffer{ width,height };

	Camera camera{
		//Vec3f{0},
		Vec3f{-2,-2,2},
		Vec3f{1,1,-1},
		16.f / 10,
		deg_to_rad(120.f)
	};

	AABB chunk_volume{ .min = {-1}, .max = {1} };


	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			float u = (static_cast<float>(x) / vec_dir_buffer.width()) * 2 - 1;
			float v = 1.0f - ((static_cast<float>(y) + 0.5f) / vec_dir_buffer.height()) * 2.0f;
			vec_dir_buffer(x, y) = camera.get_ray(u, v).direction;
		}
	}

	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			const Ray r{ camera.position(), vec_dir_buffer(x, y) , 1 / vec_dir_buffer(x, y) };
			color_buffer(x, y) = chunk_volume.intersect(r) ?  Vec3f { 1 } : Vec3f{0};
		}
	}

	PpmWriter writer{};

	writer.write("out.ppm", color_buffer);
}
*/

using namespace vrt;
int main()
{
    DagPoolManager manager{};

    Node leaf1 = { .indices = {0,2,0,0,0,0,0,0} };
    Node leaf2 = { .indices = {0,2,0,0,0,0,0,0} };
    Node leaf3 = { .indices = {0,3,0,0,0,0,0,0} };

    uint32_t idx1 = manager.AddNode(leaf1);
    std::println("Dodano leaf1 pod indeksem: {}", idx1);

    uint32_t idx2 = manager.AddNode(leaf2);
    std::println("Dodano leaf2 pod indeksem: {}", idx2);

    uint32_t idx3 = manager.AddNode(leaf3);
    std::println("Dodano leaf3 pod indeksem: {}", idx3);

    Node root = { .indices = {idx1, idx3, 0, 0, 0, 0, 0, 0} };
    uint32_t root_idx = manager.AddNode(root);
    std::println("Dodano root  pod indeksem: {}", root_idx);

    const auto& nodes = manager.dagPool().nodes;
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        std::print("Indeks [{}] -> [ ", i);
        for (uint32_t child_idx : nodes[i].indices)
        {
            std::print("{} ", child_idx);
        }
        std::print("]\n");
    }

    return 0;
}