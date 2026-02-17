#pragma once
#include <vector>
#include <cstddef>
#include <span>
#include <stdexcept>
#include <algorithm>


namespace vrt
{
	template<class T>
	class Buffer2D
	{

	public:
		Buffer2D() = default;
		Buffer2D(std::size_t width, std::size_t height, const T& init = T{})
			: width_(width), height_(height), data_(width* height, init) {}

		std::size_t width()  const noexcept { return width_; }
		std::size_t height() const noexcept { return height_; }
		std::size_t size() const noexcept { return data_.size(); }

		constexpr bool in_bounds(std::size_t x, std::size_t y) const noexcept
		{
			return x < width_ && y < height_;
		}

		constexpr std::size_t index(std::size_t x, std::size_t y) const noexcept
		{
			return x + width_ * y;
		};

		T& operator()(std::size_t x, std::size_t y) noexcept
		{
			return data_[index(x, y)];
		}

		const T& operator()(std::size_t x, std::size_t y) const noexcept
		{
			return data_[index(x, y)];
		}

		T& at(std::size_t x, std::size_t y)
		{
			if (!in_bounds(x, y)) throw std::out_of_range("Buffer2D::at out of range");
			return data_[index(x, y)];
		}

		const T& at(std::size_t x, std::size_t y) const
		{
			if (!in_bounds(x, y)) throw std::out_of_range("Buffer2D::at out of range");
			return data_[index(x, y)];
		}

		T* data() noexcept 
		{
			return data_.data(); 
		}

		const T* data() const noexcept 
		{ 
			return data_.data(); 
		}

		std::span<T> span() noexcept 
		{ 
			return data_; 
		}

		std::span<const T> span() const noexcept 
		{ 
			return data_; 
		}

		void fill(const T& v)
		{
			std::fill(data_.begin(), data_.end(), v);
		}

	private:
		std::size_t width_ = 0;
		std::size_t height_ = 0;
		std::vector<T> data_;
	};

}