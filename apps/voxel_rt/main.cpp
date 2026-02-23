#include <print>
#include "voxel_rt/core/camera.hpp"
#include "voxel_rt/math/ray.hpp"
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/math/math.hpp"
#include "voxel_rt/io/ppm_writer.hpp"
#include "voxel_rt/core/buffer_2d.hpp"
#include "voxel_rt/math/aabb.hpp"

using namespace vrt;
int main()
{
	Buffer2D<Vec3f> vec_dir_buffer{ 160,100 };
	Buffer2D<Vec3f> color_buffer{ 160,100 };

	Camera camera{
		Vec3f{-2,-2,2},
		Vec3f{1,1,-1},
		16.f / 10,
		deg_to_rad(90.f)
	};

	AABB chunk_volume{ .min = {-1}, .max = {1} };


	for (int y = 0; y < vec_dir_buffer.height(); ++y)
	{
		for (int x = 0; x < vec_dir_buffer.width(); ++x)
		{
			float u = (static_cast<float>(x) / vec_dir_buffer.width()) * 2 - 1;
			float v = 1.0f - ((static_cast<float>(y) + 0.5f) / vec_dir_buffer.height()) * 2.0f;
			vec_dir_buffer(x, y) = camera.get_ray(u, v).direction;
		}
	}

	for (int y = 0; y < color_buffer.height(); ++y)
	{
		for (int x = 0; x < color_buffer.width(); ++x)
		{
			const Ray r{ camera.position(), vec_dir_buffer(x, y) , 1 / vec_dir_buffer(x, y) };
			color_buffer(x, y) = chunk_volume.intersect(r) ?  Vec3f { 1 } : Vec3f{0};
		}
	}

	PpmWriter writer{};

	writer.write("out.ppm", color_buffer);
}