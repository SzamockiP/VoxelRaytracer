#pragma once
#include <vrt/voxel/dag_pool_manager.hpp>
#include <vrt/math/ray.hpp>
#include <vrt/core/ray_hit.hpp>

namespace vrt
{
	class Intersector
	{
	public:
		Intersector(DagPoolManager& dag_pool_manager) : dag_pool_manager_(dag_pool_manager) {};

		const RayHit intersect(const Ray& ray, std::uint32_t root_index, std::uint32_t  max_depth, const Vec3f& root_center) const noexcept;

		void set_dag_pool_manager(DagPoolManager& dag_pool_manager)
		{
			dag_pool_manager_ = dag_pool_manager;
		}

	private:
		DagPoolManager& dag_pool_manager_;
	};
}