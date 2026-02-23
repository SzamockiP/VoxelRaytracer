#pragma once
#include <vrt/voxel/dag_pool.hpp>
#include <unordered_map>
#include <vrt/voxel/node_hasher.hpp>
#include <vrt/voxel/leaf_hasher.hpp>

namespace vrt
{
	class DagPoolManager
	{
	public:
		std::uint32_t AddNode(const Node& node);
		std::uint32_t AddLeaf(const Leaf& leaf);
		void RemoveNode(std::uint32_t index);
		const DagPool& dagPool() const noexcept { return dag_pool_; }

	private:
		DagPool dag_pool_;

		std::unordered_map<Node, std::uint32_t, NodeHasher> node_indices;
		std::vector<std::uint32_t> node_refcounts;
		std::vector<std::uint32_t> free_node_indices_;

		std::unordered_map<Leaf, std::uint32_t, LeafHasher> leaf_indices;
		std::vector<std::uint32_t> leaf_refcounts;
		std::vector<std::uint32_t> free_leaf_indices_;
	};
}