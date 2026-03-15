#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <vrt/core/types.hpp>
#include <vrt/accel/blas.hpp>

namespace vrt {

	struct Instance {
		u32 root_index;
		u8 depth;
		glm::mat4 transform;
	};

	class Scene {
	public:
		void add_instance(const Instance& instance)
		{
			instances_.push_back(instance);
		};

		Blas& blas() { return blas_; };

		const std::vector<Instance>& instances() const noexcept {
			return instances_;
		}

		void debug() 
		{
			blas_.debug();
		};

	private:
		Blas blas_;
		std::vector<Instance> instances_;
	};
}