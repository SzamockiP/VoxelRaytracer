#pragma once
#include <concepts>

namespace vrt
{
	template<typename T>
	concept VoxelStorage = requires(T a)
	{
		{ a.bounds() } -> std::same_as<AABB>;
	}
}