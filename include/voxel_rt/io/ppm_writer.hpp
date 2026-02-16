#pragma once
#include <string>
#include "voxel_rt/core/image_buffer.hpp"
namespace vrt
{

class PpmWriter
{
public:
	bool write(const std::string& file_path, const ImageBuffer& image_buffer) const;
};

}