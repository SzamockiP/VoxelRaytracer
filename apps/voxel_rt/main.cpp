#include <print>
#include "voxel_rt/core/camera.hpp"
#include "voxel_rt/math/ray.hpp"
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/math/math.hpp"
#include "voxel_rt/io/ppm_writer.hpp"
#include "voxel_rt/core/buffer_2d.hpp"

using namespace vrt;
int main()
{
	Buffer2D<Vec3f> image_buffer{ 160,100 };

	Camera camera{
		Vec3f{0},
		Vec3f{1},
		16.f / 10,
		deg_to_rad(90.f)
	};

	for (int y = 0; y < image_buffer.height(); ++y)
	{
		for (int x = 0; x < image_buffer.width(); ++x)
		{
			float u = (static_cast<float>(x) / image_buffer.width()) * 2 - 1;
			float v = 1.0f - ((static_cast<float>(y) + 0.5f) / image_buffer.height()) * 2.0f;
			image_buffer(x, y) = camera.get_ray(u, v).direction;
		}
	}

	PpmWriter writer{};

	writer.write("out.ppm", image_buffer);
}