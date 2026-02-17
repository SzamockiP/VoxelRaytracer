#pragma once
#include <string>
#include "voxel_rt/core/image_buffer.hpp"
#include "voxel_rt/core/buffer_2d.hpp"
namespace vrt
{

class PpmWriter
{
public:
	bool write(const std::string& file_path, const Buffer2D<Vec3>& image_buffer) const;
	bool write(const std::string& file_path, const ImageBuffer& image_buffer) const;
};

}