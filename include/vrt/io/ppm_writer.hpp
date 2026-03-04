#pragma once
#include <string>
#include <vrt/core/buffer_2d.hpp>
#include <vrt/core/types.hpp>
#include <glm/glm.hpp>

namespace vrt
{

class PpmWriter
{
public:
	bool write(const std::string& file_path, const Buffer2D<glm::vec3>& image_buffer) const;
};

}