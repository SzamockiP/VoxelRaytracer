#pragma once
#include <glm/glm.hpp>
#include <vrt/core/types.hpp>
#include <vrt/voxel/voxel.hpp>

namespace vrt
{
	struct Hit
	{
		float t;
		glm::vec3 normal;
		Voxel voxel;
	};
}