//
// Created by birdpeople on 8/9/2023.
//

#ifndef VECTOR_TYPE_T_H
#define VECTOR_TYPE_T_H


#include <cuda_runtime.h>
#include "typedef.h"
#include <thrust/extrema.h>
#include <vector_types.h>
#include <vector_functions.hpp>
#include <vector_functions.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

template <typename T>
CUDA_INLINE_CALLABLE constexpr T min(const T& a, const T& b)
{
    return b < a ? b : a;
}

template <typename T>
CUDA_INLINE_CALLABLE constexpr T max(const T& a, const T& b)
{
    return b > a ? b : a;
}

template <typename T>
CUDA_INLINE_CALLABLE constexpr T clamp(const T& v, const T& minv, const T& maxv)
{
    return min(max(v, minv), maxv);
}

template <typename T>
CUDA_INLINE_CALLABLE T pow2(const T& x)
{
    return x * x;
}

template <typename T>
CUDA_INLINE_CALLABLE T pow3(const T& x)
{
    return x * x * x;
}

using unit = uint32_t;

//define types and functions on the host corresponding to the CUDA vector types
/*
 #ifndef __CUDACC__
    struct alignas(8) int2 {
        int32_t x, y;
        constexpr int2(int32_t v = 0) : x(v), y(v) {}
        constexpr int2(int32_t xx, int32_t yy) : x(xx), y(yy) {}
    };
    inline constexpr int2 make_int2(int32_t x, int32_t y) {
        return int2(x, y);
    }
    struct int3 {
        int32_t x, y, z;
        constexpr int3(int32_t v = 0) : x(v), y(v), z(v) {}
        constexpr int3(int32_t xx, int32_t yy, int32_t zz) : x(xx), y(yy), z(zz) {}
    };
    inline constexpr int3 make_int3(int32_t x, int32_t y, int32_t z) {
        return int3(x, y, z);
    }
    struct alignas(16) int4 {
        int32_t x, y, z, w;
        constexpr int4(int32_t v = 0) : x(v), y(v), z(v), w(v) {}
        constexpr int4(int32_t xx, int32_t yy, int32_t zz, int32_t ww) : x(xx), y(yy), z(zz), w(ww) {}
    };
    inline constexpr int4 make_int4(int32_t x, int32_t y, int32_t z, int32_t w) {
        return int4(x, y, z, w);
    }
    struct alignas(8) uint2 {
        uint32_t x, y;
        constexpr uint2(uint32_t v = 0) : x(v), y(v) {}
        constexpr uint2(uint32_t xx, uint32_t yy) : x(xx), y(yy) {}
    };
    inline constexpr uint2 make_uint2(uint32_t x, uint32_t y) {
        return uint2(x, y);
    }
    struct uint3 {
        uint32_t x, y, z;
        constexpr uint3(uint32_t v = 0) : x(v), y(v), z(v) {}
        constexpr uint3(uint32_t xx, uint32_t yy, uint32_t zz) : x(xx), y(yy), z(zz) {}
    };
    inline constexpr uint3 make_uint3(uint32_t x, uint32_t y, uint32_t z) {
        return uint3(x, y, z);
    }
    struct uint4 {
        uint32_t x, y, z, w;
        constexpr uint4(uint32_t v = 0) : x(v), y(v), z(v), w(v) {}
        constexpr uint4(uint32_t xx, uint32_t yy, uint32_t zz, uint32_t ww) : x(xx), y(yy), z(zz), w(ww) {}
    };
    inline constexpr uint4 make_uint4(uint32_t x, uint32_t y, uint32_t z, uint32_t w) {
        return uint4(x, y, z, w);
    }
    struct alignas(8) float2 {
        float x, y;
        constexpr float2(float v = 0) : x(v), y(v) {}
        constexpr float2(float xx, float yy) : x(xx), y(yy) {}
    };
    inline float2 make_float2(float x, float y) {
        return float2(x, y);
    }
    struct float3 {
        float x, y, z;
        constexpr float3(float v = 0) : x(v), y(v), z(v) {}
        constexpr float3(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
    };
    inline constexpr float3 make_float3(float x, float y, float z) {
        return float3(x, y, z);
    }
    struct alignas(16) float4 {
        float x, y, z, w;
        constexpr float4(float v = 0) : x(v), y(v), z(v), w(v) {}
        constexpr float4(float xx, float yy, float zz, float ww) : x(xx), y(yy), z(zz), w(ww) {}
    };
    inline constexpr float4 make_float4(float x, float y, float z, float w) {
        return float4(x, y, z, w);
    }
#endif
*/
CUDA_INLINE_CALLABLE float3 getXYZ(const float4& v)
{
    return make_float3(v.x, v.y, v.z);
}

CUDA_INLINE_CALLABLE double3 getXYZ(const double4& v)
{
    return make_double3(v.x, v.y, v.z);
}

CUDA_INLINE_CALLABLE int2 make_int2(const float2& v)
{
    return make_int2(static_cast<int32_t>(v.x), static_cast<int32_t>(v.y));
}
CUDA_INLINE_CALLABLE int2 make_int2(const int3& v)
{
    return make_int2(v.x, v.y);
}
CUDA_INLINE_CALLABLE int2 make_int2(const uint3& v)
{
    return make_int2(static_cast<int32_t>(v.x), static_cast<int32_t>(v.y));
}
//inline constexpr bool operator==(const int2& v0, const int2& v1)
//{
//    return v0.x == v1.x && v0.y == v1.y;
//}

CUDA_INLINE_CALLABLE bool operator!=(const int2& v0, const uint2& v1)
{
    return v0.x != v1.x || v0.y != v1.y;
}
CUDA_INLINE_CALLABLE uint2 operator+(const int2& v0, const uint2& v1)
{
    return make_uint2(v0.x + v1.x, v0.y + v1.y);
}
CUDA_INLINE_CALLABLE int2 operator+(const int2& v0, const int2& v1)
{
    return make_int2(v0.x + v1.x, v0.y + v1.y);
}
CUDA_INLINE_CALLABLE int2 operator*(const int2& v0, const int2& v1)
{
    return make_int2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE int2 operator*(uint32_t s, const int2& v)
{
    return make_int2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE int2 operator*(const int2& v, uint32_t s)
{
    return make_int2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE int2& operator*=(int2& v0, const int2& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    return v0;
}
CUDA_INLINE_CALLABLE int2& operator*=(int2& v, uint32_t s)
{
    v.x *= s;
    v.y *= s;
    return v;
}
CUDA_INLINE_CALLABLE int2 operator/(const int2& v0, const int2& v1)
{
    return make_int2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE int2 operator/(const int2& v, uint32_t s)
{
    return make_int2(v.x / s, v.y / s);
}
CUDA_INLINE_CALLABLE uint2 operator/(const int2& v0, const uint2& v1)
{
    return make_uint2(v0.x / v1.x, v0.y / v1.y);
}

CUDA_INLINE_CALLABLE uint2 make_uint2(const float2& v)
{
    return make_uint2(static_cast<uint32_t>(v.x), static_cast<uint32_t>(v.y));
}
CUDA_INLINE_CALLABLE uint2 make_uint2(const int3& v)
{
    return make_uint2(static_cast<uint32_t>(v.x), static_cast<uint32_t>(v.y));
}
CUDA_INLINE_CALLABLE uint2 make_uint2(const uint3& v)
{
    return make_uint2(v.x, v.y);
}
CUDA_INLINE_CALLABLE bool operator==(const uint2& v0, const uint2& v1)
{
    return v0.x == v1.x && v0.y == v1.y;
}
CUDA_INLINE_CALLABLE bool operator!=(const uint2& v0, const uint2& v1)
{
    return v0.x != v1.x || v0.y != v1.y;
}
CUDA_INLINE_CALLABLE bool operator==(const uint2& v0, const int2& v1)
{
    return v0.x == v1.x && v0.y == v1.y;
}
CUDA_INLINE_CALLABLE bool operator!=(const uint2& v0, const int2& v1)
{
    return v0.x != v1.x || v0.y != v1.y;
}
CUDA_INLINE_CALLABLE uint2 operator+(const uint2& v0, const uint2& v1)
{
    return make_uint2(v0.x + v1.x, v0.y + v1.y);
}
CUDA_INLINE_CALLABLE uint2& operator+=(uint2& v, uint32_t s)
{
    v.x += s;
    v.y += s;
    return v;
}
CUDA_INLINE_CALLABLE uint2 operator-(const uint2& v, uint32_t s)
{
    return make_uint2(v.x - s, v.y - s);
}
CUDA_INLINE_CALLABLE uint2& operator-=(uint2& v, uint32_t s)
{
    v.x -= s;
    v.y -= s;
    return v;
}
CUDA_INLINE_CALLABLE uint2 operator*(const uint2& v0, const uint2& v1)
{
    return make_uint2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE uint2 operator*(float s, const uint2& v)
{
    return make_uint2(static_cast<uint32_t>(s * v.x), static_cast<uint32_t>(s * v.y));
}
CUDA_INLINE_CALLABLE uint2 operator*(const uint2& v, float s)
{
    return make_uint2(static_cast<uint32_t>(s * v.x), static_cast<uint32_t>(s * v.y));
}
CUDA_INLINE_CALLABLE uint2& operator*=(uint2& v0, const uint2& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    return v0;
}
CUDA_INLINE_CALLABLE uint2& operator*=(uint2& v, uint32_t s)
{
    v.x *= s;
    v.y *= s;
    return v;
}
CUDA_INLINE_CALLABLE uint2 operator/(const uint2& v0, const uint2& v1)
{
    return make_uint2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE uint2 operator/(const uint2& v0, const int2& v1)
{
    return make_uint2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE uint2 operator/(const uint2& v, uint32_t s)
{
    return make_uint2(v.x / s, v.y / s);
}
CUDA_INLINE_CALLABLE uint2& operator/=(uint2& v, uint32_t s)
{
    v.x /= s;
    v.y /= s;
    return v;
}
CUDA_INLINE_CALLABLE uint2 operator%(const uint2& v0, const uint2& v1)
{
    return make_uint2(v0.x % v1.x, v0.y % v1.y);
}
CUDA_INLINE_CALLABLE uint2 operator<<(const uint2& v, uint32_t s)
{
    return make_uint2(v.x << s, v.y << s);
}
CUDA_INLINE_CALLABLE uint2& operator<<=(uint2& v, uint32_t s)
{
    v = v << s;
    return v;
}
CUDA_INLINE_CALLABLE uint2 operator>>(const uint2& v, uint32_t s)
{
    return make_uint2(v.x >> s, v.y >> s);
}
CUDA_INLINE_CALLABLE uint2& operator>>=(uint2& v, uint32_t s)
{
    v = v >> s;
    return v;
}

CUDA_INLINE_CALLABLE float2 make_float2(float v)
{
    return make_float2(v, v);
}
CUDA_INLINE_CALLABLE float2 make_float2(const int2& v)
{
    return make_float2(static_cast<float>(v.x), static_cast<float>(v.y));
}
CUDA_INLINE_CALLABLE float2 make_float2(const uint2& v)
{
    return make_float2(static_cast<float>(v.x), static_cast<float>(v.y));
}
CUDA_INLINE_CALLABLE bool operator==(const float2& v0, const float2& v1)
{
    return v0.x == v1.x && v0.y == v1.y;
}
CUDA_INLINE_CALLABLE bool operator!=(const float2& v0, const float2& v1)
{
    return v0.x != v1.x || v0.y != v1.y;
}
CUDA_INLINE_CALLABLE float2 operator-(const float2& v)
{
    return make_float2(-v.x, -v.y);
}
CUDA_INLINE_CALLABLE float2 operator+(const float2& v0, const float2& v1)
{
    return make_float2(v0.x + v1.x, v0.y + v1.y);
}
CUDA_INLINE_CALLABLE float2 operator-(const float2& v0, const float2& v1)
{
    return make_float2(v0.x - v1.x, v0.y - v1.y);
}
CUDA_INLINE_CALLABLE float2 operator*(const float2& v0, const float2& v1)
{
    return make_float2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE float2 operator*(float s, const float2& v)
{
    return make_float2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE float2 operator*(const float2& v, float s)
{
    return make_float2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE float2& operator*=(float2& v, float s)
{
    v = v * s;
    return v;
}
CUDA_INLINE_CALLABLE float2 operator*(const int2& v0, const float2& v1)
{
    return make_float2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE float2 operator*(const float2& v0, const int2& v1)
{
    return make_float2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE float2 operator/(const float2& v0, const float2& v1)
{
    return make_float2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE float2 operator/(const float2& v0, const int2& v1)
{
    return make_float2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE float2 operator/(const float2& v, float s)
{
    float r = 1 / s;
    return r * v;
}
CUDA_INLINE_CALLABLE float2& operator/=(float2& v, float s)
{
    v = v / s;
    return v;
}

CUDA_INLINE_CALLABLE float3 make_float3(float v)
{
    return make_float3(v, v, v);
}
CUDA_INLINE_CALLABLE float3 make_float3(const float4& v)
{
    return make_float3(v.x, v.y, v.z);
}
CUDA_INLINE_CALLABLE bool operator==(const float3& v0, const float3& v1)
{
    return v0.x == v1.x && v0.y == v1.y && v0.z == v1.z;
}
CUDA_INLINE_CALLABLE bool operator!=(const float3& v0, const float3& v1)
{
    return v0.x != v1.x || v0.y != v1.y || v0.z != v1.z;
}
CUDA_INLINE_CALLABLE float3 operator-(const float3& v)
{
    return make_float3(-v.x, -v.y, -v.z);
}
CUDA_INLINE_CALLABLE float3 operator+(const float3& v0, const float3& v1)
{
    return make_float3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}
CUDA_INLINE_CALLABLE float3& operator+=(float3& v0, const float3& v1)
{
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE float3 operator-(const float3& v0, const float3& v1)
{
    return make_float3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}
CUDA_INLINE_CALLABLE float3& operator-=(float3& v0, const float3& v1)
{
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE float3 operator*(const float3& v0, const float3& v1)
{
    return make_float3(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z);
}
CUDA_INLINE_CALLABLE float3 operator*(float s, const float3& v)
{
    return make_float3(s * v.x, s * v.y, s * v.z);
}
CUDA_INLINE_CALLABLE float3 operator*(const float3& v, float s)
{
    return make_float3(s * v.x, s * v.y, s * v.z);
}
CUDA_INLINE_CALLABLE float3& operator*=(float3& v0, const float3& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    v0.z *= v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE float3& operator*=(float3& v, float s)
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}
CUDA_INLINE_CALLABLE float3 operator/(const float3& v0, const float3& v1)
{
    return make_float3(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z);
}
CUDA_INLINE_CALLABLE float3 operator/(const float3& v, float s)
{
    float r = 1 / s;
    return r * v;
}
CUDA_INLINE_CALLABLE float3 safeDivide(const float3& v0, const float3& v1)
{
    return make_float3(v1.x != 0.0f ? v0.x / v1.x : 0.0f,
                       v1.y != 0.0f ? v0.y / v1.y : 0.0f,
                       v1.z != 0.0f ? v0.z / v1.z : 0.0f);
}
CUDA_INLINE_CALLABLE float3 safeDivide(const float3& v, float d)
{
    return d != 0.0f ? (v / d) : make_float3(0.0f);
}
CUDA_INLINE_CALLABLE float3& operator/=(float3& v, float s)
{
    float r = 1 / s;
    return v *= r;
}
CUDA_INLINE_CALLABLE bool allFinite(const float3& v)
{
#if !defined(__CUDA_ARCH__)
    using std::isfinite;
#endif
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

CUDA_INLINE_CALLABLE float4 make_float4(float v)
{
    return make_float4(v, v, v, v);
}
CUDA_INLINE_CALLABLE float4 make_float4(const float3& v)
{
    return make_float4(v.x, v.y, v.z, 0.0f);
}
CUDA_INLINE_CALLABLE float4 make_float4(const float3& v, float w)
{
    return make_float4(v.x, v.y, v.z, w);
}
CUDA_INLINE_CALLABLE bool operator==(const float4& v0, const float4& v1)
{
    return v0.x == v1.x && v0.y == v1.y && v0.z == v1.z && v0.w == v1.w;
}
CUDA_INLINE_CALLABLE bool operator!=(const float4& v0, const float4& v1)
{
    return v0.x != v1.x || v0.y != v1.y || v0.z != v1.z || v0.w != v1.w;
}
CUDA_INLINE_CALLABLE float4 operator-(const float4& v)
{
    return make_float4(-v.x, -v.y, -v.z, -v.w);
}
CUDA_INLINE_CALLABLE float4 operator+(const float4& v0, const float4& v1)
{
    return make_float4(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w);
}
CUDA_INLINE_CALLABLE float4& operator+=(float4& v0, const float4& v1)
{
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
    v0.w += v1.w;
    return v0;
}
CUDA_INLINE_CALLABLE float4 operator-(const float4& v0, const float4& v1)
{
    return make_float4(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z, v0.w - v1.w);
}
CUDA_INLINE_CALLABLE float4& operator-=(float4& v0, const float4& v1)
{
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
    v0.w -= v1.w;
    return v0;
}
CUDA_INLINE_CALLABLE float4 operator*(const float4& v0, const float4& v1)
{
    return make_float4(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z, v0.w * v1.w);
}
CUDA_INLINE_CALLABLE float4 operator*(float s, const float4& v)
{
    return make_float4(s * v.x, s * v.y, s * v.z, s * v.w);
}
CUDA_INLINE_CALLABLE float4 operator*(const float4& v, float s)
{
    return make_float4(s * v.x, s * v.y, s * v.z, s * v.w);
}
CUDA_INLINE_CALLABLE float4& operator*=(float4& v0, const float4& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    v0.z *= v1.z;
    v0.w *= v1.w;
    return v0;
}
CUDA_INLINE_CALLABLE float4& operator*=(float4& v, float s)
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
    v.w *= s;
    return v;
}
CUDA_INLINE_CALLABLE float4 operator/(const float4& v0, const float4& v1)
{
    return make_float4(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z, v0.w / v1.w);
}
CUDA_INLINE_CALLABLE float4 operator/(const float4& v, float s)
{
    float r = 1 / s;
    return r * v;
}
CUDA_INLINE_CALLABLE float4& operator/=(float4& v, float s)
{
    float r = 1 / s;
    return v *= r;
}
CUDA_INLINE_CALLABLE bool allFinite(const float4& v)
{
#if !defined(__CUDA_ARCH__)
    using std::isfinite;
#endif
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z) && isfinite(v.w);
}

CUDA_INLINE_CALLABLE double2 make_double2(double v)
{
    return make_double2(v, v);
}
CUDA_INLINE_CALLABLE double2 make_double2(const int2& v)
{
    return make_double2(static_cast<double>(v.x), static_cast<double>(v.y));
}
CUDA_INLINE_CALLABLE double2 make_double2(const uint2& v)
{
    return make_double2(static_cast<double>(v.x), static_cast<double>(v.y));
}
CUDA_INLINE_CALLABLE bool operator==(const double2& v0, const double2& v1)
{
    return v0.x == v1.x && v0.y == v1.y;
}
CUDA_INLINE_CALLABLE bool operator!=(const double2& v0, const double2& v1)
{
    return v0.x != v1.x || v0.y != v1.y;
}
CUDA_INLINE_CALLABLE double2 operator-(const double2& v)
{
    return make_double2(-v.x, -v.y);
}
CUDA_INLINE_CALLABLE double2 operator+(const double2& v0, const double2& v1)
{
    return make_double2(v0.x + v1.x, v0.y + v1.y);
}
CUDA_INLINE_CALLABLE double2 operator-(const double2& v0, const double2& v1)
{
    return make_double2(v0.x - v1.x, v0.y - v1.y);
}
CUDA_INLINE_CALLABLE double2 operator*(const double2& v0, const double2& v1)
{
    return make_double2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE double2 operator*(double s, const double2& v)
{
    return make_double2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE double2 operator*(const double2& v, double s)
{
    return make_double2(s * v.x, s * v.y);
}
CUDA_INLINE_CALLABLE double2& operator*=(double2& v, double s)
{
    v = v * s;
    return v;
}
CUDA_INLINE_CALLABLE double2 operator*(const int2& v0, const double2& v1)
{
    return make_double2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE double2 operator*(const double2& v0, const int2& v1)
{
    return make_double2(v0.x * v1.x, v0.y * v1.y);
}
CUDA_INLINE_CALLABLE double2 operator/(const double2& v0, const double2& v1)
{
    return make_double2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE double2 operator/(const double2& v0, const int2& v1)
{
    return make_double2(v0.x / v1.x, v0.y / v1.y);
}
CUDA_INLINE_CALLABLE double2 operator/(const double2& v, double s)
{
    double r = 1 / s;
    return r * v;
}
CUDA_INLINE_CALLABLE double2& operator/=(double2& v, double s)
{
    v = v / s;
    return v;
}

CUDA_INLINE_CALLABLE double3 make_double3(double v)
{
    return make_double3(v, v, v);
}
CUDA_INLINE_CALLABLE double3 make_double3(const double4& v)
{
    return make_double3(v.x, v.y, v.z);
}
CUDA_INLINE_CALLABLE bool operator==(const double3& v0, const double3& v1)
{
    return v0.x == v1.x && v0.y == v1.y && v0.z == v1.z;
}
CUDA_INLINE_CALLABLE bool operator!=(const double3& v0, const double3& v1)
{
    return v0.x != v1.x || v0.y != v1.y || v0.z != v1.z;
}
CUDA_INLINE_CALLABLE double3 operator-(const double3& v)
{
    return make_double3(-v.x, -v.y, -v.z);
}
CUDA_INLINE_CALLABLE double3 operator+(const double3& v0, const double3& v1)
{
    return make_double3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z);
}
CUDA_INLINE_CALLABLE double3& operator+=(double3& v0, const double3& v1)
{
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE double3 operator-(const double3& v0, const double3& v1)
{
    return make_double3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z);
}
CUDA_INLINE_CALLABLE double3& operator-=(double3& v0, const double3& v1)
{
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE double3 operator*(const double3& v0, const double3& v1)
{
    return make_double3(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z);
}
CUDA_INLINE_CALLABLE double3 operator*(double s, const double3& v)
{
    return make_double3(s * v.x, s * v.y, s * v.z);
}
CUDA_INLINE_CALLABLE double3 operator*(const double3& v, double s)
{
    return make_double3(s * v.x, s * v.y, s * v.z);
}
CUDA_INLINE_CALLABLE double3& operator*=(double3& v0, const double3& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    v0.z *= v1.z;
    return v0;
}
CUDA_INLINE_CALLABLE double3& operator*=(double3& v, double s)
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
    return v;
}
CUDA_INLINE_CALLABLE double3 operator/(const double3& v0, const double3& v1)
{
    return make_double3(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z);
}
CUDA_INLINE_CALLABLE double3 operator/(const double3& v, double s)
{
    double r = 1 / s;
    return r * v;
}
CUDA_INLINE_CALLABLE double3 safeDivide(const double3& v0, const double3& v1)
{
    return make_double3(v1.x != 0.0f ? v0.x / v1.x : 0.0f,
                        v1.y != 0.0f ? v0.y / v1.y : 0.0f,
                        v1.z != 0.0f ? v0.z / v1.z : 0.0f);
}
CUDA_INLINE_CALLABLE double3 safeDivide(const double3& v, double d)
{
    return d != 0.0f ? (v / d) : make_double3(0.0f);
}
CUDA_INLINE_CALLABLE double3& operator/=(double3& v, double s)
{
    double r = 1 / s;
    return v *= r;
}
CUDA_INLINE_CALLABLE bool allFinite(const double3& v)
{
#if !defined(__CUDA_ARCH__)
    using std::isfinite;
#endif
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

CUDA_INLINE_CALLABLE double4 make_double4(double v)
{
    return make_double4(v, v, v, v);
}
CUDA_INLINE_CALLABLE double4 make_double4(const double3& v)
{
    return make_double4(v.x, v.y, v.z, 0.0f);
}
CUDA_INLINE_CALLABLE double4 make_double4(const double3& v, double w)
{
    return make_double4(v.x, v.y, v.z, w);
}
CUDA_INLINE_CALLABLE bool operator==(const double4& v0, const double4& v1)
{
    return v0.x == v1.x && v0.y == v1.y && v0.z == v1.z && v0.w == v1.w;
}
CUDA_INLINE_CALLABLE bool operator!=(const double4& v0, const double4& v1)
{
    return v0.x != v1.x || v0.y != v1.y || v0.z != v1.z || v0.w != v1.w;
}

CUDA_INLINE_CALLABLE double4 operator-(const double4& v)
{
    return make_double4(-v.x, -v.y, -v.z, -v.w);
}

CUDA_INLINE_CALLABLE double4 operator+(const double4& v0, const double4& v1)
{
    return make_double4(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w);
}

CUDA_INLINE_CALLABLE double4& operator+=(double4& v0, const double4& v1)
{
    v0.x += v1.x;
    v0.y += v1.y;
    v0.z += v1.z;
    v0.w += v1.w;
    return v0;
}

CUDA_INLINE_CALLABLE double4 operator-(const double4& v0, const double4& v1)
{
    return make_double4(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z, v0.w - v1.w);
}

CUDA_INLINE_CALLABLE double4& operator-=(double4& v0, const double4& v1)
{
    v0.x -= v1.x;
    v0.y -= v1.y;
    v0.z -= v1.z;
    v0.w -= v1.w;
    return v0;
}

CUDA_INLINE_CALLABLE double4 operator*(const double4& v0, const double4& v1)
{
    return make_double4(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z, v0.w * v1.w);
}

CUDA_INLINE_CALLABLE double4 operator*(double s, const double4& v)
{
    return make_double4(s * v.x, s * v.y, s * v.z, s * v.w);
}

CUDA_INLINE_CALLABLE double4 operator*(const double4& v, double s)
{
    return make_double4(s * v.x, s * v.y, s * v.z, s * v.w);
}

CUDA_INLINE_CALLABLE double4& operator*=(double4& v0, const double4& v1)
{
    v0.x *= v1.x;
    v0.y *= v1.y;
    v0.z *= v1.z;
    v0.w *= v1.w;
    return v0;
}

CUDA_INLINE_CALLABLE double4& operator*=(double4& v, double s)
{
    v.x *= s;
    v.y *= s;
    v.z *= s;
    v.w *= s;
    return v;
}

CUDA_INLINE_CALLABLE double4 operator/(const double4& v0, const double4& v1)
{
    return make_double4(v0.x / v1.x, v0.y / v1.y, v0.z / v1.z, v0.w / v1.w);
}

CUDA_INLINE_CALLABLE double4 operator/(const double4& v, double s)
{
    double r = 1 / s;
    return r * v;
}

CUDA_INLINE_CALLABLE double4& operator/=(double4& v, double s)
{
    double r = 1 / s;
    return v *= r;
}

CUDA_INLINE_CALLABLE bool allFinite(const double4& v)
{
#if !defined(__CUDA_ARCH__)
    using std::isfinite;
#endif
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z) && isfinite(v.w);
}

CUDA_INLINE_CALLABLE int2 min(const int2& v0, const int2& v1)
{
    return make_int2(min(v0.x, v1.x), min(v0.y, v1.y));
}

CUDA_INLINE_CALLABLE int2 max(const int2& v0, const int2& v1)
{
    return make_int2(max(v0.x, v1.x), max(v0.y, v1.y));
}

CUDA_INLINE_CALLABLE uint2 min(const uint2& v0, const uint2& v1)
{
    return make_uint2(min(v0.x, v1.x), min(v0.y, v1.y));
}
CUDA_INLINE_CALLABLE uint2 max(const uint2& v0, const uint2& v1)
{
    return make_uint2(max(v0.x, v1.x), max(v0.y, v1.y));
}

CUDA_INLINE_CALLABLE float2 min(const float2& v0, const float2& v1)
{
    return make_float2(min(v0.x, v1.x), min(v0.y, v1.y));
}
CUDA_INLINE_CALLABLE float2 max(const float2& v0, const float2& v1)
{
    return make_float2(max(v0.x, v1.x), max(v0.y, v1.y));
}
CUDA_INLINE_CALLABLE float cross(const float2& v0, const float2& v1)
{
    return v0.x * v1.y - v0.y * v1.x;
}

CUDA_INLINE_CALLABLE float3 min(const float3& v0, const float3& v1)
{
    return make_float3(fmin(v0.x, v1.x), fmin(v0.y, v1.y), fmin(v0.z, v1.z));
}
CUDA_INLINE_CALLABLE float3 max(const float3& v0, const float3& v1)
{
    return make_float3(fmax(v0.x, v1.x), fmax(v0.y, v1.y), fmax(v0.z, v1.z));
}
CUDA_INLINE_CALLABLE float dot(const float3& v0, const float3& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}
CUDA_INLINE_CALLABLE float3 cross(const float3& v0, const float3& v1)
{
    return make_float3(v0.y * v1.z - v0.z * v1.y,
                       v0.z * v1.x - v0.x * v1.z,
                       v0.x * v1.y - v0.y * v1.x);
}
CUDA_INLINE_CALLABLE float squaredDistance(const float3& p0, const float3& p1)
{
    float3 d = p1 - p0;
    return dot(d, d);
}
CUDA_INLINE_CALLABLE float length(const float3& v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}
CUDA_INLINE_CALLABLE float squaredLength(const float3& v)
{
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
CUDA_INLINE_CALLABLE float3 normalize(const float3& v)
{
    return v / length(v);
}

CUDA_INLINE_CALLABLE float4 min(const float4& v0, const float4& v1)
{
    return make_float4(
        fmin(v0.x, v1.x), fmin(v0.y, v1.y), fmin(v0.z, v1.z), fmin(v0.w, v1.w));
}
CUDA_INLINE_CALLABLE float4 max(const float4& v0, const float4& v1)
{
    return make_float4(
        fmax(v0.x, v1.x), fmax(v0.y, v1.y), fmax(v0.z, v1.z), fmax(v0.w, v1.w));
}
CUDA_INLINE_CALLABLE float dot(const float4& v0, const float4& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w;
}

CUDA_INLINE_CALLABLE int32_t floatToOrderedInt(float fVal)
{
#if defined(__CUDA_ARCH__)
    int32_t iVal = __float_as_int(fVal);
#else
    int32_t iVal = *reinterpret_cast<int32_t*>(&fVal);
#endif
    return (iVal >= 0) ? iVal : iVal ^ 0x7FFFFFFF;
}

CUDA_INLINE_CALLABLE float orderedIntToFloat(int32_t iVal)
{
    int32_t orgVal = (iVal >= 0) ? iVal : iVal ^ 0x7FFFFFFF;
#if defined(__CUDA_ARCH__)
    return __int_as_float(orgVal);
#else
    return *reinterpret_cast<float*>(&orgVal);
#endif
}


struct float3AsOrderedInt
{
    int32_t x, y, z;

    CUDA_CALLABLE float3AsOrderedInt()
        : x(0)
        , y(0)
        , z(0)
    {
    }
    CUDA_CALLABLE float3AsOrderedInt(const float3& v)
        : x(floatToOrderedInt(v.x))
        , y(floatToOrderedInt(v.y))
        , z(floatToOrderedInt(v.z))
    {
    }

    CUDA_CALLABLE explicit operator float3() const
    {
        return make_float3(orderedIntToFloat(x), orderedIntToFloat(y), orderedIntToFloat(z));
    }
};

CUDA_INLINE_CALLABLE double2 min(const double2& v0, const double2& v1)
{
    return make_double2(min(v0.x, v1.x), min(v0.y, v1.y));
}

CUDA_INLINE_CALLABLE double2 max(const double2& v0, const double2& v1)
{
    return make_double2(max(v0.x, v1.x), max(v0.y, v1.y));
}

CUDA_INLINE_CALLABLE double cross(const double2& v0, const double2& v1)
{
    return v0.x * v1.y - v0.y * v1.x;
}

CUDA_INLINE_CALLABLE double3 min(const double3& v0, const double3& v1)
{
    return make_double3(fmin(v0.x, v1.x), fmin(v0.y, v1.y), fmin(v0.z, v1.z));
}

CUDA_INLINE_CALLABLE double3 max(const double3& v0, const double3& v1)
{
    return make_double3(fmax(v0.x, v1.x), fmax(v0.y, v1.y), fmax(v0.z, v1.z));
}

CUDA_INLINE_CALLABLE double dot(const double3& v0, const double3& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

CUDA_INLINE_CALLABLE double3 cross(const double3& v0, const double3& v1)
{
    return make_double3(v0.y * v1.z - v0.z * v1.y,
                        v0.z * v1.x - v0.x * v1.z,
                        v0.x * v1.y - v0.y * v1.x);
}

CUDA_INLINE_CALLABLE double squaredDistance(const double3& p0, const double3& p1)
{
    double3 d = p1 - p0;
    return dot(d, d);
}

CUDA_INLINE_CALLABLE double length(const double3& v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

CUDA_INLINE_CALLABLE double squaredLength(const double3& v)
{
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

CUDA_INLINE_CALLABLE double3 normalize(const double3& v)
{
    return v / length(v);
}

CUDA_INLINE_CALLABLE double4 min(const double4& v0, const double4& v1)
{
    return make_double4(
        fmin(v0.x, v1.x), fmin(v0.y, v1.y), fmin(v0.z, v1.z), fmin(v0.w, v1.w));
}

CUDA_INLINE_CALLABLE double4 max(const double4& v0, const double4& v1)
{
    return make_double4(
        fmax(v0.x, v1.x), fmax(v0.y, v1.y), fmax(v0.z, v1.z), fmax(v0.w, v1.w));
}

CUDA_INLINE_CALLABLE double dot(const double4& v0, const double4& v1)
{
    return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z + v0.w * v1.w;
}

CUDA_INLINE_CALLABLE int64_t doubleToOrderedLongLong(double fVal)
{
#if defined(__CUDA_ARCH__)
    int64_t iVal = __double_as_longlong(fVal);
#else
    int64_t iVal = *reinterpret_cast<int64_t*>(&fVal);
#endif
    return (iVal >= 0) ? iVal : iVal ^ 0xFFFFFFFF;
}

CUDA_INLINE_CALLABLE double orderedLongLongToDouble(int64_t iVal)
{
    int64_t orgVal = (iVal >= 0) ? iVal : iVal ^ 0xFFFFFFFF;
#if defined(__CUDA_ARCH__)
    return __longlong_as_double(orgVal);
#else
    return *reinterpret_cast<double*>(&orgVal);
#endif
}

struct double3AsOrderedLongLong
{
    int64_t x, y, z;

    CUDA_CALLABLE double3AsOrderedLongLong()
        : x(0)
        , y(0)
        , z(0)
    {
    }
    CUDA_CALLABLE double3AsOrderedLongLong(const double3& v)
        : x(doubleToOrderedLongLong(v.x))
        , y(doubleToOrderedLongLong(v.y))
        , z(doubleToOrderedLongLong(v.z))
    {
    }

    CUDA_CALLABLE explicit operator double3() const
    {
        return make_double3(orderedLongLongToDouble(x),
                            orderedLongLongToDouble(y),
                            orderedLongLongToDouble(z));
    }
};

#endif
