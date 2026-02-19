#pragma once
#include "voxel_rt/math/aabb.hpp"
#include "voxel_rt/core/voxel_type.hpp"
#include <stdexcept>
#include <vector>

namespace vrt
{
	class VoxelGrid
	{
	public:
		VoxelGrid(std::size_t resolution, const Vec3i& center = Vec3i{0}, VoxelType init = VoxelType::EMPTY) :
			resolution_(validate_resolution(resolution)),
			center_(center),
			data_(resolution_* resolution_* resolution_, init),
			bounds_(make_bounds(resolution_, center_))
		{};

		VoxelType& operator()(std::size_t x, std::size_t y, std::size_t z)
		{
			return data_[index(x,y,z)];
		}

		const VoxelType& operator()(std::size_t x, std::size_t y, std::size_t z) const
		{
			return data_[index(x, y, z)];
		}

		constexpr const AABB& bounds() const noexcept
		{
			return bounds_;
		};

	private:
		std::size_t resolution_;
		Vec3i center_;
		std::vector<VoxelType> data_;
		AABB bounds_;

		static std::size_t validate_resolution(std::size_t resolution)
		{
			if (!is_pow2(resolution))
				throw std::invalid_argument("[vrt::VoxelGrid::VoxelGrid] resolution must be a power of 2");
			return resolution;
		}

		static constexpr bool is_pow2(std::size_t x) noexcept
		{
			return x != 0 && (x & (x - 1)) == 0;
		}

		static AABB make_bounds(std::size_t resolution, const Vec3i& center) noexcept
		{
			float half = static_cast<float>(resolution) * 0.5f;
			return {
				Vec3f{-half} + center,
				Vec3f{half} + center
			};
		}

		constexpr std::size_t index(std::size_t x, std::size_t y, std::size_t z) const noexcept
		{
			return x + resolution_ * (y + resolution_ * z);
		}
	};
}