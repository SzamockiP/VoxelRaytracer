#pragma once
#include <vrt/accel/blas/blas_manager.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/rt/hit.hpp>

namespace vrt
{
	class Intersector
	{
	public:
		Intersector(BlasManager& blas_manager) : blas_manager_(blas_manager) {};

		const Hit intersect(const Ray& ray, std::uint32_t root_index, std::uint32_t  max_depth, const Vec3f& root_center) const noexcept;

		void set_blas_manager(BlasManager& blas_manager)
		{
			blas_manager_ = blas_manager;
		}

	private:
		BlasManager& blas_manager_;
	};
}