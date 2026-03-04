#pragma once
#include <vrt/core/types.hpp>
#include <vrt/accel/blas/blas.hpp>
#include <vrt/accel/blas/node_hasher.hpp>
#include <vrt/accel/blas/leaf_hasher.hpp>
#include <unordered_map>
#include <limits>


namespace vrt
{

	class BlasManager
	{
	public:
		u32 AddNode(const Node& node);
		u32 AddLeaf(const Leaf& leaf);
		void RemoveNode(u32 index);
		const Blas& dagPool() const noexcept { return dag_pool_; }

	private:
		Blas dag_pool_;

		std::unordered_map<Node, u32, NodeHasher> node_indices;
		std::vector<u32> node_refcounts;
		std::vector<u32> free_node_indices_;

		std::unordered_map<Leaf, u32, LeafHasher> leaf_indices;
		std::vector<u32> leaf_refcounts;
		std::vector<u32> free_leaf_indices_;
	};

	const u32 EMPTY = std::numeric_limits<unsigned int>::max();
}