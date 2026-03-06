#pragma once
#include <vrt/core/types.hpp>
#include <vrt/core/scene.hpp>
#include <vrt/math/ray.hpp>
#include <glm/glm.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	class Intersector
	{
	public:
		Intersector(Scene& scene) : scene_(scene) {};

		const Hit intersect(const Ray& ray, const Instance& instance) const noexcept;

	private:
		Scene& scene_;
	};
}