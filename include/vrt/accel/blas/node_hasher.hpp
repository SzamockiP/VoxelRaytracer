#pragma once

namespace vrt
{
	struct NodeHasher
	{
		std::size_t operator()(const Node& node) const noexcept
		{
			std::size_t hash = 0;
			for (int i = 0; i < 8; i++)
			{
				hash ^= std::hash<u32>{}(node.indices[i]) << 1;
			}
			return hash;
		}
	};
}