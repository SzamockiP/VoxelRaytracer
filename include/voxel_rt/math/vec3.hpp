#pragma once
#include <cmath>


namespace vrt {

struct Vec3
{
	float x{}, y{}, z{};


	float lenght2()
	{
		return x * x + y * y + z * z;
	}

	float lenght()
	{
		return std::sqrtf(lenght2());
	}

	Vec3 unit()
	{
		float len = lenght();
		return Vec3{ x / len, y / len, z / len };
	}

	Vec3 operator+(Vec3& other)
	{
		return Vec3{ x + other.x, y + other.y, z + other.z };
	}

	Vec3 operator-(Vec3& other)
	{
		return Vec3{ x - other.x, y - other.y, z - other.z };
	}
};

}

