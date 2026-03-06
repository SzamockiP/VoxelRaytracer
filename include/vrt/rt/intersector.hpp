#pragma once
#include <vrt/core/types.hpp>
#include <vrt/accel/blas/blas.hpp>
#include <vrt/math/ray.hpp>
#include <glm/glm.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	class Intersector
	{
	public:
		Intersector(Blas& blas) : blas_(blas) {};

		const Hit intersect(const Ray& ray, u32 root_index, u32  max_depth, const glm::vec3& root_center) const noexcept;

		void set_blas_manager(Blas& blas)
		{
			blas_ = blas;
		}

	private:
		Blas& blas_;
	};
}