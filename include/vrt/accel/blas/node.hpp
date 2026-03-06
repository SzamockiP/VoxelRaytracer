#pragma once
#include <vrt/core/types.hpp>

namespace vrt
{
	struct Node
	{
		u32 indices[8];

		bool operator==(const Node& other) const noexcept
		{
			for (int i = 0; i < 8; i++)
			{
				if(indices[i] != other.indices[i])
					return false;
			}
			return true;
		}
	};
}

template <>
struct std::hash<vrt::Node>
{
	std::size_t operator()(const vrt::Node& node) const noexcept
	{
		std::size_t hash = 0;
		for (int i = 0; i < 8; i++)
		{
			std::size_t h = std::hash<vrt::u32>{}(static_cast<vrt::u32>(node.indices[i]));
			hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};