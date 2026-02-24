#pragma once
#include <string>
#include <vrt/core/buffer_2d.hpp>
#include <vrt/math/Vec3.hpp>

namespace vrt
{

class PpmWriter
{
public:
	bool write(const std::string& file_path, const Buffer2D<Vec3f>& image_buffer) const;
};

}