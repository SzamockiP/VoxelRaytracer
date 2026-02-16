#pragma once
#include <vector>
#include <cstddef>

#include "voxel_rt/math/vec3.hpp"

namespace vrt
{

class ImageBuffer
{

public:
	ImageBuffer(int width, int height);

	int width()  const noexcept { return width_; }
	int height() const noexcept { return height_; }
	const std::vector<Vec3>& pixels() const noexcept { return pixels_; };

	void set_pixel(int x, int y, Vec3 color) noexcept;


private:
	std::vector<Vec3> pixels_;
	const int width_;
	const int height_;

	std::size_t index(int x, int y) const noexcept;
};

}