#pragma once
#include <vrt/math/vec3.hpp>
#include <vrt/voxel/voxel.hpp>

namespace vrt
{
	struct Hit
	{
		float t;
		Vec3f normal;
		Voxel voxel;
	};
}