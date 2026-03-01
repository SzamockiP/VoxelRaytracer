#pragma once
#include <cstdint>

namespace vrt
{
	enum class Voxel: std::uint32_t
	{
		EMPTY,
		FULL
	};
}