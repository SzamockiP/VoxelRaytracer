#pragma once
#include <vrt/accel/blas/blas_manager.hpp>
#include <glm/glm.hpp>

namespace vrt
{

	class BlasBuilder
	{
	public:
		BlasBuilder(BlasManager& blas_manager) : blas_manager_(blas_manager) {};
		u32 BuildTree(glm::vec3 center, u32 size);
		void SetBlasManager(BlasManager& blas_manager) { blas_manager_ = blas_manager; };
	private:
		BlasManager& blas_manager_;
	};
}