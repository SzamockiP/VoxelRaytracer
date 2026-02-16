#include "voxel_rt/io/ppm_writer.hpp"
#include <fstream>
#include <algorithm>

bool vrt::PpmWriter::write(const std::string& file_path, const ImageBuffer& image_buffer) const
{
	std::ofstream out(file_path, std::ios::binary);
	if (!out) return false;

	out << "P6\n" << image_buffer.width() << '\n' << image_buffer.height() << "\n255\n";

	for (const auto& pixel : image_buffer.pixels())
	{
		const unsigned char rgb[3] = {

			static_cast<unsigned char>(std::clamp(pixel.x * 255 + 0.5f , 0.f, 255.f)),
			static_cast<unsigned char>(std::clamp(pixel.y * 255 + 0.5f, 0.f, 255.f)),
			static_cast<unsigned char>(std::clamp(pixel.z * 255 + 0.5f, 0.f, 255.f))
		};
		out.write(reinterpret_cast<const char*>(rgb), 3);
	}

	return static_cast<bool>(out);
}