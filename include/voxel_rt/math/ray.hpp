#pragma once
#include "voxel_rt/math/vec3.hpp"

namespace vrt
{
	struct Ray
	{
		Vec3f origin;
		Vec3f direction;
		Vec3f direction_inverse;
	};
}