#pragma once
#include <string>
#include "voxel_rt/core/buffer_2d.hpp"
#include "voxel_rt/math/Vec3.hpp"

namespace vrt
{

class PpmWriter
{
public:
	bool write(const std::string& file_path, const Buffer2D<Vec3>& image_buffer) const;
};

}