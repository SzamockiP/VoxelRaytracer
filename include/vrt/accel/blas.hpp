#pragma once
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
#include <vrt/core/types.hpp>
#include <array>
#include <vrt/voxel/voxel.hpp>
#include <print>

namespace vrt
{	
	class Blas
	{
	public:
		struct Node
		{
			std::array<u32, 8> indices;
			bool operator==(const Node&) const noexcept = default;
		};

		struct Leaf
		{
			std::array<Voxel, 8> voxels;
			bool operator==(const Leaf&) const noexcept = default;
		};

		u32 add_node(const Node& node);
		u32 add_leaf(const Leaf& leaf);

		u32 build(glm::vec3 center, u8 resolution);

		const std::vector<Node>& nodes() const noexcept{
			return nodes_;
		}

		const std::vector<Leaf>& leaves() const noexcept {
			return leaves_;
		}

		void print_debug() {
			std::println("unique leaves: {}", leaves_.size());
			std::println("unique nodes:  {}", nodes_.size());
			std::println("leaf map size: {}", leaf_indices_.size());
			std::println("node map size: {}", node_indices_.size());
		}
	private:
		struct NodeHasher
		{
			std::size_t operator()(const Node& node) const noexcept;
		};

		struct LeafHasher {
			std::size_t operator()(const Leaf& leaf) const noexcept;
		};

		std::vector<Node> nodes_;
		std::unordered_map<Node, u32, NodeHasher> node_indices_;

		std::vector<u32> node_refcounts_;
		std::vector<u32> free_node_indices_;

		std::vector<Leaf> leaves_;
		std::unordered_map<Leaf, u32, LeafHasher> leaf_indices_;

		std::vector<u32> leaf_refcounts_;
		std::vector<u32> free_leaf_indices_;
	};

	const u32 EMPTY = std::numeric_limits<unsigned int>::max();

}
