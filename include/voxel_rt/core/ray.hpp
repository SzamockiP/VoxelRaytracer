#pragma once
#include "voxel_rt/math/vec3.hpp"

namespace vrt
{
struct Ray
{
	Vec3 origin;
	Vec3 direction;
	Vec3 inv_direction;
};
}