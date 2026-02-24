#pragma once
#include <vrt/voxel/voxel.hpp>

namespace vrt
{
	struct Leaf
	{
		std::uint32_t voxels[8];

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