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

DagPoolManager manager{};
int shape(const Vec3f& pos)
{
	return length(pos) <= 120 ? 1 : 0;
}

std::uint32_t build_tree(Vec3i min, Vec3i max)
{
	Vec3i size = max - min;
	Vec3i half_size = size / 2;
	Vec3i center = min + half_size;

	if (max - min == Vec3{2})
	{
		Leaf l{ 
			shape(center - Vec3f{ -0.5f, -0.5f, -0.5f }),
			shape(center - Vec3f{  0.5f, -0.5f, -0.5f }),
			shape(center - Vec3f{ -0.5f,  0.5f, -0.5f }),
			shape(center - Vec3f{  0.5f,  0.5f, -0.5f }),
			shape(center - Vec3f{ -0.5f, -0.5f,  0.5f }),
			shape(center - Vec3f{  0.5f, -0.5f,  0.5f }),
			shape(center - Vec3f{ -0.5f,  0.5f,  0.5f }),
			shape(center - Vec3f{  0.5f,  0.5f,  0.5f })
		};

		return manager.AddLeaf(l);
	}
	else
	{
		Vec3i right = { half_size.x, 0, 0 };
		Vec3i up	   = { 0, half_size.y, 0 };
		Vec3i back  = { 0, 0, half_size.z };
		Node n = {
			build_tree(min, min + half_size),
			build_tree(min + right, max - back - up),
			build_tree(min + up, max - back - right),
			build_tree(min + right + up, max - back),
			build_tree(min + back, max - right - up),
			build_tree(min + back + right, max - up),
			build_tree(min + up + back, max - right),
			build_tree(min + half_size, max)
		};
		return manager.AddNode(n);
	}
}

int main()
{
	int root_idx = build_tree(Vec3i{ -128 }, Vec3i{ 128 });
	int root_idx2 = build_tree(Vec3i{ -64 }, Vec3i{ 64 });

	std::println("Root index: {}", root_idx);
	std::println("Root index: {}", root_idx2);
	std::println("Leaves: {}", manager.dagPool().leaves.size() );
	std::println("Nodes:  {}", manager.dagPool().nodes.size());

    return 0;
}