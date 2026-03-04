#pragma once
#include <glm/glm.hpp>
#include <vrt/core/types.hpp>

namespace vrt
{
	struct Chunk
	{
		u32 root_index;
		glm::ivec3 position;
	};
}
