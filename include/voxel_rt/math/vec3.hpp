#pragma once
#include <cmath>


namespace vrt {

    struct Vec3
    {
        float x{}, y{}, z{};

        constexpr Vec3() = default;
        constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

        constexpr Vec3 operator+(Vec3 v) const noexcept { return { x + v.x, y + v.y, z + v.z }; }
        constexpr Vec3 operator-(Vec3 v) const noexcept { return { x - v.x, y - v.y, z - v.z }; }
        constexpr Vec3 operator*(float a) const noexcept { return { x * a, y * a, z * a }; }
        constexpr Vec3 operator/(float a) const noexcept { return { x / a, y / a, z / a }; }

        constexpr Vec3& operator+=(Vec3 v) noexcept { x += v.x; y += v.y; z += v.z; return *this; }
        constexpr Vec3& operator-=(Vec3 v) noexcept { x -= v.x; y -= v.y; z -= v.z; return *this; }
        constexpr Vec3& operator*=(float a) noexcept { x *= a; y *= a; z *= a; return *this; }
        constexpr Vec3& operator/=(float a) noexcept { x /= a; y /= a; z /= a; return *this; }
    };
}

