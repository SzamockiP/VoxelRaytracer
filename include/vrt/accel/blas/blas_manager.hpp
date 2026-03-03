#pragma once
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
		std::uint32_t AddNode(const Node& node);
		std::uint32_t AddLeaf(const Leaf& leaf);
		void RemoveNode(std::uint32_t index);
		const Blas& dagPool() const noexcept { return dag_pool_; }

	private:
		Blas dag_pool_;

		std::unordered_map<Node, std::uint32_t, NodeHasher> node_indices;
		std::vector<std::uint32_t> node_refcounts;
		std::vector<std::uint32_t> free_node_indices_;

		std::unordered_map<Leaf, std::uint32_t, LeafHasher> leaf_indices;
		std::vector<std::uint32_t> leaf_refcounts;
		std::vector<std::uint32_t> free_leaf_indices_;
	};

	const std::uint32_t EMPTY = std::numeric_limits<unsigned int>::max();
}