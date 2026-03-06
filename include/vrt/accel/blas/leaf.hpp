#pragma once
#include <vrt/voxel/voxel.hpp>
#include <vrt/core/types.hpp>

namespace vrt
{
	struct Leaf
	{
		Voxel voxels[8];

		bool operator==(const Leaf& other) const noexcept
		{
			for (int i = 0; i < 8; i++)
			{
				if (voxels[i] != other.voxels[i])
					return false;
			}
			return true;
		}
	};
}

template <>
struct std::hash<vrt::Leaf>
{
	std::size_t operator()(const vrt::Leaf& leaf) const noexcept
	{
		std::size_t hash = 0;
		for (int i = 0; i < 8; i++)
		{
			std::size_t h = std::hash<vrt::u32>{}(static_cast<vrt::u32>(leaf.voxels[i]));
			hash ^= h + 0x9e3779b9 + (hash << 6) + (hash >> 2);
		}
		return hash;
	}
};