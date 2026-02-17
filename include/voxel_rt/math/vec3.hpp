#pragma once
#include <concepts>
#include <type_traits>


namespace vrt {

    template<typename T>
    concept Arithmetic = std::is_arithmetic_v<T>;

    template<Arithmetic T = float>
    struct Vec3
    {
        T x{}, y{}, z{};

        constexpr Vec3() = default;
        constexpr Vec3(T s) : x(s), y(s), z(s) {}
        constexpr Vec3(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {}

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

        friend constexpr auto operator+(const Vec3& lhs, Arithmetic auto rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x + rhs, lhs.y + rhs, lhs.z + rhs };
        }

        friend constexpr auto operator+(Arithmetic auto lhs, const Vec3& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs + rhs.x, lhs + rhs.y, lhs + rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator-(const Vec3& lhs, const Vec3<U>& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        friend constexpr auto operator-(const Vec3& lhs, Arithmetic auto rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x - rhs, lhs.y - rhs, lhs.z - rhs };
        }

        friend constexpr auto operator-(Arithmetic auto lhs, const Vec3& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs - rhs.x, lhs - rhs.y, lhs - rhs.z };
        }

        template<Arithmetic U>
        friend constexpr auto operator*(const Vec3& lhs, const Vec3<U>& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z };
        }

        friend constexpr auto operator*(const Vec3& lhs, Arithmetic auto rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x * rhs, lhs.y * rhs, lhs.z * rhs };
        }

        friend constexpr auto operator*(Arithmetic auto lhs, const Vec3& rhs) noexcept
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs * rhs.x, lhs * rhs.y, lhs * rhs.z };
        }
        
        template<Arithmetic U>
        friend constexpr auto operator/(const Vec3& lhs, const Vec3<U>& rhs) 
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z };
        }

        friend constexpr auto operator/(const Vec3& lhs, Arithmetic auto rhs) 
        {
            using R = std::common_type_t<T, U>;
            return Vec3<R>{ lhs.x / rhs, lhs.y / rhs, lhs.z / rhs };
        }

        friend constexpr auto operator/(Arithmetic auto lhs, const Vec3& rhs) 
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
}

