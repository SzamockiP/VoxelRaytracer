#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <vrt/core/types.hpp>
#include <vrt/core/instance.hpp>
#include <vrt/accel/blas.hpp>
#include <vrt/accel/tlas.hpp>

namespace vrt {
	class Scene {
	public:
		void add_instance(const Instance& instance)
		{
			instances_.push_back(instance);
		};

		Blas& blas() { return blas_; };
		Tlas& tlas() { return tlas_; };

		const std::vector<Instance>& instances() const noexcept {
			return instances_;
		}

		void print_debug() 
		{
			blas_.print_debug();
		};

	private:
		Blas blas_;
		Tlas tlas_;
		std::vector<Instance> instances_;
	};
}