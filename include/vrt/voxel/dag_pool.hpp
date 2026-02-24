#pragma once
#include <vector>
#include <vrt/voxel/node.hpp>
#include <vrt/voxel/leaf.hpp>

namespace vrt 
{	
	struct DagPool
	{
		std::vector<Node> nodes;
		std::vector<Leaf> leaves;
	};
}
