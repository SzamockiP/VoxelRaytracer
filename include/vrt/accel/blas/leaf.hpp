#pragma once
#include <vrt/voxel/voxel.hpp>
#include <vrt/core/types.hpp>
#include <array>

namespace vrt
{
	struct Leaf
	{
		std::array<Voxel, 8> voxels;

		bool operator==(const Leaf&) const noexcept = default;
	};
}

template <>
struct std::hash<vrt::Leaf>
{
	std::size_t operator()(const vrt::Leaf& leaf) const noexcept
	{
		std::size_t hash = 0;
		for (auto voxel : leaf.voxels)
		{
			std::size_t h = std::hash<vrt::u32>{}(static_cast<vrt::u32>(voxel));
			hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};