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