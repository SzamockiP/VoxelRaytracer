#pragma once
#include <vrt/core/types.hpp>
#include <vrt/math/aabb.hpp>

namespace vrt{
	class Tlas {
	public:
		struct Node {
			AABB aabb;

			u32 left_child, right_child; // top bit 0 - node, 1 - leaf
			bool is_left_leaf() const noexcept { return left & 0x80000000; };
			bool is_right_leaf() const noexcept { return right & 0x80000000; };

			u32 left_index() cosnt noexcept { return left & 0x7fffffff; };
			u32 right_index() cosnt noexcept { return right & 0x7fffffff; };

			void set_left_leaf(u32 instance_index) { left_child = instance_index | 0x80000000; };
			void set_left_node(u32 node_index) { left_child = node_index; };

			void set_right_leaf(u32 instance_index) { right_child = instance_index | 0x80000000; };
			void set_right_node(u32 node_index) { right_child = node_index;	}
		};

	private:

	};
}