#pragma once
#include <vector>
#include <vrt/accel/blas/node.hpp>
#include <vrt/accel/blas/leaf.hpp>

namespace vrt
{	
	struct Blas
	{
		std::vector<Node> nodes;
		std::vector<Leaf> leaves;
	};
}
