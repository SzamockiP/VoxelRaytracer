#pragma once
#include <vrt/core/types.hpp>
#include <array>

namespace vrt
{
	struct Node
	{
		std::array<u32,8> indices;

		bool operator==(const Node&) const noexcept = default;
	};
}

template <>
struct std::hash<vrt::Node>
{
	std::size_t operator()(const vrt::Node& node) const noexcept
	{
		std::size_t hash = 0;
		for (auto index : node.indices)
		{
			std::size_t h = std::hash<vrt::u32>{}(index);
			hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};