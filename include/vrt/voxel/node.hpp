#pragma once
#include <cstdint>

namespace vrt
{
	struct Node
	{
		std::uint32_t indices[8];

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