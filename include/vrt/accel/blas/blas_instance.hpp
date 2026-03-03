#pragma once
#include <vrt/math/vec3.hpp>

namespace vrt
{
	struct Chunk
	{
		std::uint32_t root_index;
		Vec3i position;
	};
}
