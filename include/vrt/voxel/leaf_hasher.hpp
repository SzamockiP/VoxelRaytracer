#pragma once
#include <functional>

namespace vrt
{
	struct LeafHasher
	{
		std::size_t operator()(const Leaf& leaf) const noexcept
		{
			std::size_t hash = 0;
			for (int i = 0; i < 8; i++)
			{
				hash ^= std::hash<std::uint32_t>{}(leaf.voxels[i]) << 1;
			}
			return hash;
		}
	};
}