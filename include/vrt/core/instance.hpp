#pragma once
#include <glm/glm.hpp>
#include <vrt/core/types.hpp>

namespace vrt {
	struct Instance {
		u32 root_index;
		u8 depth;
		glm::mat4 transform;
	};
}