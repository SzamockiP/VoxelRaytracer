#pragma once
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
#include <vrt/core/types.hpp>
#include <vrt/accel/blas/node.hpp>
#include <vrt/accel/blas/leaf.hpp>


namespace vrt
{	
	class Blas
	{
	public:
		u32 AddNode(const Node& node);
		u32 AddLeaf(const Leaf& leaf);

		u32 Build(glm::vec3 center, u32 size);

		const std::vector<Node>& Nodes() const noexcept{
			return nodes_;
		}

		const std::vector<Leaf>& Leaves() const noexcept {
			return leaves_;
		}

	private:
		std::vector<Node> nodes_;
		std::unordered_map<Node, u32> node_indices_;

		std::vector<u32> node_refcounts_;
		std::vector<u32> free_node_indices_;

		std::vector<Leaf> leaves_;
		std::unordered_map<Leaf, u32> leaf_indices_;

		std::vector<u32> leaf_refcounts_;
		std::vector<u32> free_leaf_indices_;
	};

	const u32 EMPTY = std::numeric_limits<unsigned int>::max();

}
