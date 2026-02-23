#pragma once
#include <concepts>
#include <type_traits>
#include <cmath>


namespace vrt {

    template<typename T>
    concept Arithmetic = std::is_arithmetic_v<T>;

    template<Arithmetic T>
    struct Vec3
    {
        T x{}, y{}, z{};

        constexpr Vec3() = default;
        constexpr Vec3(T s) : x(s), y(s), z(s) {}
        constexpr Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

        constexpr T operator[](std::size_t i) const noexcept
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        constexpr T& operator[](std::size_t i) noexcept
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        constexpr bool operator==(const Vec3&) const = default;

        constexpr auto operator-() const noexcept
        {
            return Vec3{
                -x,
                -y,
                -z
            };
        }   

        template<Arithmetic U>
        friend constexpr auto operator+(const Vec3& lhs, const Vec3<U>& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator+(const Vec3& lhs, U rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x + rhs, lhs.y + rhs, lhs.z + rhs };
        }

        template<Arithmetic U>
        friend constexpr auto operator+(U lhs, const Vec3& rhs) noexcept
        {
            return rhs + lhs;
        }

        template<Arithmetic U>
        friend constexpr auto operator-(const Vec3& lhs, const Vec3<U>& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator-(const Vec3& lhs, U rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x - rhs, lhs.y - rhs, lhs.z - rhs };
        }

        template<Arithmetic U>
        friend constexpr auto operator-(U lhs, const Vec3& rhs) noexcept
        {
            return rhs - lhs;
        }

        template<Arithmetic U>
        friend constexpr auto operator*(const Vec3& lhs, const Vec3<U>& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator*(const Vec3& lhs, U rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x * rhs, lhs.y * rhs, lhs.z * rhs };
        }

        template<Arithmetic U>
        friend constexpr auto operator*(U lhs, const Vec3& rhs) noexcept
        {
            return rhs * lhs;
        }
        
        template<Arithmetic U>
        friend constexpr auto operator/(const Vec3& lhs, const Vec3<U>& rhs) 
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator/(const Vec3& lhs, U rhs) 
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x / rhs, lhs.y / rhs, lhs.z / rhs };
        }

        template<Arithmetic U>
        friend constexpr auto operator/(U lhs, const Vec3& rhs) 
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs / rhs.x, lhs / rhs.y, lhs / rhs.z };
        }


        template<Arithmetic U>
        constexpr Vec3& operator+=(const Vec3<U>& v) noexcept
        {
            x += static_cast<T>(v.x);
            y += static_cast<T>(v.y);
            z += static_cast<T>(v.z);
            return *this; 
        }

        template<Arithmetic U>
        constexpr Vec3& operator-=(const Vec3<U>& v) noexcept
        {
            x -= static_cast<T>(v.x);
            y -= static_cast<T>(v.y);
            z -= static_cast<T>(v.z);
            return *this;
        }

        template<Arithmetic U>
        constexpr Vec3& operator*=(const Vec3<U>& v) noexcept
        {
            x *= static_cast<T>(v.x);
            y *= static_cast<T>(v.y);
            z *= static_cast<T>(v.z);
            return *this;
        }

        template<Arithmetic U>
        constexpr Vec3& operator/=(const Vec3<U>& v)
        {
            x /= static_cast<T>(v.x);
            y /= static_cast<T>(v.y);
            z /= static_cast<T>(v.z);
            return *this;
        }
    };

    template<class T, class U, class V>
    Vec3(T, U, V) -> Vec3<std::common_type_t<T, U, V>>;

    using Vec3f = Vec3<float>;
    using Vec3i = Vec3<int>;
    using Vec3d = Vec3<double>;

    template<Arithmetic T>
    inline constexpr auto length_sq(const Vec3<T>& v) noexcept
    {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    template<Arithmetic T>
    inline const auto length(const Vec3<T>& v) noexcept
        requires std::floating_point<T>
    {
        return std::sqrt(length_sq(v));
    }

    template<Arithmetic T, Arithmetic U>
    inline constexpr auto dot(const Vec3<T>& a, const Vec3<U>& b) noexcept
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    template<Arithmetic T, Arithmetic U>
    inline constexpr auto cross(const Vec3<T>& a, const Vec3<U>& b) noexcept
    {
        return Vec3{
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    template<Arithmetic T>
    inline constexpr auto normalize(const Vec3<T>& v) noexcept
        requires std::floating_point<T>
    {
        T len = length(v);
        return (len > 0.f) ? Vec3<T>{ v.x / len, v.y / len, v.z / len } : Vec3<T>{};
    }
}

