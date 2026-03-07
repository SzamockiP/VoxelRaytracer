#pragma once
#include <vrt/core/types.hpp>
#include <vrt/core/instance.hpp>
#include <vrt/math/aabb.hpp>
#include <vrt/accel/blas.hpp>

namespace vrt{
	class Tlas {
	public:
		struct Node {
			AABB aabb;

			u32 left_child, right_child; // top bit 0 - node, 1 - leaf
			bool is_left_leaf() const noexcept { return left_child & 0x80000000; };
			bool is_right_leaf() const noexcept { return right_child & 0x80000000; };

			u32 left_index() const noexcept { return left_child & 0x7fffffff; };
			u32 right_index() const noexcept { return right_child & 0x7fffffff; };

			void set_left_leaf(u32 instance_index) { left_child = instance_index | 0x80000000; };
			void set_left_node(u32 node_index) { left_child = node_index; };

			void set_right_leaf(u32 instance_index) { right_child = instance_index | 0x80000000; };
			void set_right_node(u32 node_index) { right_child = node_index;	}
		};

		const std::vector<Node>& nodes() const noexcept {
			return nodes_;
		}

		void build(const std::vector<Instance>& instances);

	private:
		struct MortonEntry {
			u32 index;
			u32 code;
		};

		std::vector<Node> nodes_;
		std::vector<MortonEntry> morton_entries_;

		void compute_morton_entries(const std::vector<Instance>& instances);
	};
}