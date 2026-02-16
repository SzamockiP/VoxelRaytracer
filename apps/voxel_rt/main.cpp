#include <print>
#include "voxel_rt/core/image_buffer.hpp"
#include "voxel_rt/math/vec3.hpp"
#include "voxel_rt/io/ppm_writer.hpp"

int main()
{
	std::print("voxel_rt: hello");
	vrt::ImageBuffer ib{ 720,480 };
	for (int y = 0; y < ib.height(); ++y)
	{
		for (int x = 0; x < ib.width(); ++x)
		{
			vrt::Vec3 color = {
				0,
				static_cast<float>(x) / ib.width(),
				static_cast<float>(y) / ib.height(),
			};

			ib.set_pixel(x, y, color);
		}
	}

	vrt::PpmWriter writer{};

	writer.write("out.ppm", ib);
}