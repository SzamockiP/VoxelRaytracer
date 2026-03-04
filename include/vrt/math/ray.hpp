#pragma once
#include <vrt/core/types.hpp>
#include <glm/glm.hpp>


namespace vrt
{
	struct Ray
	{
		glm::vec3 origin;
		glm::vec3 direction;
		glm::vec3 direction_inverse;
	};
}