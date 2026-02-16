#include "voxel_rt/core/image_buffer.hpp"
#include <cassert>

vrt::ImageBuffer::ImageBuffer(int width, int height) :
	width_(width),
	height_(height),
	pixels_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height))
{
	assert(width > 0 && height > 0);
}

std::size_t vrt::ImageBuffer::index(int x, int y) const noexcept
{
	assert(0 <= x && x < width_);
	assert(0 <= y && y < height_);

	return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
}

void vrt::ImageBuffer::set_pixel(int x, int y, Vec3 color) noexcept
{
	assert(0 <= x && x < width_);
	assert(0 <= y && y < height_);

	pixels_[index(x, y)] = color;
}
