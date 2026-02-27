#pragma once
#include <muda/ext/eigen/eigen_core_cxx20.h>
#include <vector_types.h>
namespace muda
{
namespace eigen
{
    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<float, 2, 1>> as_eigen(float2& val)
    {
        return Eigen::Map<Eigen::Matrix<float, 2, 1>>(reinterpret_cast<float*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<float, 2, 1>> as_eigen(const float2& val)
    {
        return Eigen::Map<const Eigen::Matrix<float, 2, 1>>(
            reinterpret_cast<const float*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<float, 3, 1>> as_eigen(float3& val)
    {
        return Eigen::Map<Eigen::Matrix<float, 3, 1>>(reinterpret_cast<float*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<float, 3, 1>> as_eigen(const float3& val)
    {
        return Eigen::Map<const Eigen::Matrix<float, 3, 1>>(
            reinterpret_cast<const float*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<float, 4, 1>> as_eigen(float4& val)
    {
        return Eigen::Map<Eigen::Matrix<float, 4, 1>>(reinterpret_cast<float*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<float, 4, 1>> as_eigen(const float4& val)
    {
        return Eigen::Map<const Eigen::Matrix<float, 4, 1>>(
            reinterpret_cast<const float*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<double, 2, 1>> as_eigen(double2& val)
    {
        return Eigen::Map<Eigen::Matrix<double, 2, 1>>(reinterpret_cast<double*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<double, 2, 1>> as_eigen(const double2& val)
    {
        return Eigen::Map<const Eigen::Matrix<double, 2, 1>>(
            reinterpret_cast<const double*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<double, 3, 1>> as_eigen(double3& val)
    {
        return Eigen::Map<Eigen::Matrix<double, 3, 1>>(reinterpret_cast<double*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<double, 3, 1>> as_eigen(const double3& val)
    {
        return Eigen::Map<const Eigen::Matrix<double, 3, 1>>(
            reinterpret_cast<const double*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<double, 4, 1>> as_eigen(double4& val)
    {
        return Eigen::Map<Eigen::Matrix<double, 4, 1>>(reinterpret_cast<double*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<double, 4, 1>> as_eigen(const double4& val)
    {
        return Eigen::Map<const Eigen::Matrix<double, 4, 1>>(
            reinterpret_cast<const double*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<int, 2, 1>> as_eigen(int2& val)
    {
        return Eigen::Map<Eigen::Matrix<int, 2, 1>>(reinterpret_cast<int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<int, 2, 1>> as_eigen(const int2& val)
    {
        return Eigen::Map<const Eigen::Matrix<int, 2, 1>>(
            reinterpret_cast<const int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<int, 3, 1>> as_eigen(int3& val)
    {
        return Eigen::Map<Eigen::Matrix<int, 3, 1>>(reinterpret_cast<int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<int, 3, 1>> as_eigen(const int3& val)
    {
        return Eigen::Map<const Eigen::Matrix<int, 3, 1>>(
            reinterpret_cast<const int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<int, 4, 1>> as_eigen(int4& val)
    {
        return Eigen::Map<Eigen::Matrix<int, 4, 1>>(reinterpret_cast<int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<int, 4, 1>> as_eigen(const int4& val)
    {
        return Eigen::Map<const Eigen::Matrix<int, 4, 1>>(
            reinterpret_cast<const int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned int, 2, 1>> as_eigen(uint2& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned int, 2, 1>>(
            reinterpret_cast<unsigned int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned int, 2, 1>> as_eigen(const uint2& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned int, 2, 1>>(
            reinterpret_cast<const unsigned int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned int, 3, 1>> as_eigen(uint3& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned int, 3, 1>>(
            reinterpret_cast<unsigned int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned int, 3, 1>> as_eigen(const uint3& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned int, 3, 1>>(
            reinterpret_cast<const unsigned int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned int, 4, 1>> as_eigen(uint4& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned int, 4, 1>>(
            reinterpret_cast<unsigned int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned int, 4, 1>> as_eigen(const uint4& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned int, 4, 1>>(
            reinterpret_cast<const unsigned int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<short, 2, 1>> as_eigen(short2& val)
    {
        return Eigen::Map<Eigen::Matrix<short, 2, 1>>(reinterpret_cast<short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<short, 2, 1>> as_eigen(const short2& val)
    {
        return Eigen::Map<const Eigen::Matrix<short, 2, 1>>(
            reinterpret_cast<const short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<short, 3, 1>> as_eigen(short3& val)
    {
        return Eigen::Map<Eigen::Matrix<short, 3, 1>>(reinterpret_cast<short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<short, 3, 1>> as_eigen(const short3& val)
    {
        return Eigen::Map<const Eigen::Matrix<short, 3, 1>>(
            reinterpret_cast<const short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<short, 4, 1>> as_eigen(short4& val)
    {
        return Eigen::Map<Eigen::Matrix<short, 4, 1>>(reinterpret_cast<short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<short, 4, 1>> as_eigen(const short4& val)
    {
        return Eigen::Map<const Eigen::Matrix<short, 4, 1>>(
            reinterpret_cast<const short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned short, 2, 1>> as_eigen(ushort2& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned short, 2, 1>>(
            reinterpret_cast<unsigned short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned short, 2, 1>> as_eigen(const ushort2& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned short, 2, 1>>(
            reinterpret_cast<const unsigned short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned short, 3, 1>> as_eigen(ushort3& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned short, 3, 1>>(
            reinterpret_cast<unsigned short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned short, 3, 1>> as_eigen(const ushort3& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned short, 3, 1>>(
            reinterpret_cast<const unsigned short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned short, 4, 1>> as_eigen(ushort4& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned short, 4, 1>>(
            reinterpret_cast<unsigned short*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned short, 4, 1>> as_eigen(const ushort4& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned short, 4, 1>>(
            reinterpret_cast<const unsigned short*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<char, 2, 1>> as_eigen(char2& val)
    {
        return Eigen::Map<Eigen::Matrix<char, 2, 1>>(reinterpret_cast<char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<char, 2, 1>> as_eigen(const char2& val)
    {
        return Eigen::Map<const Eigen::Matrix<char, 2, 1>>(
            reinterpret_cast<const char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<char, 3, 1>> as_eigen(char3& val)
    {
        return Eigen::Map<Eigen::Matrix<char, 3, 1>>(reinterpret_cast<char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<char, 3, 1>> as_eigen(const char3& val)
    {
        return Eigen::Map<const Eigen::Matrix<char, 3, 1>>(
            reinterpret_cast<const char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<char, 4, 1>> as_eigen(char4& val)
    {
        return Eigen::Map<Eigen::Matrix<char, 4, 1>>(reinterpret_cast<char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<char, 4, 1>> as_eigen(const char4& val)
    {
        return Eigen::Map<const Eigen::Matrix<char, 4, 1>>(
            reinterpret_cast<const char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned char, 2, 1>> as_eigen(uchar2& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned char, 2, 1>>(
            reinterpret_cast<unsigned char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned char, 2, 1>> as_eigen(const uchar2& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned char, 2, 1>>(
            reinterpret_cast<const unsigned char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned char, 3, 1>> as_eigen(uchar3& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned char, 3, 1>>(
            reinterpret_cast<unsigned char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned char, 3, 1>> as_eigen(const uchar3& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned char, 3, 1>>(
            reinterpret_cast<const unsigned char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned char, 4, 1>> as_eigen(uchar4& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned char, 4, 1>>(
            reinterpret_cast<unsigned char*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned char, 4, 1>> as_eigen(const uchar4& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned char, 4, 1>>(
            reinterpret_cast<const unsigned char*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long int, 2, 1>> as_eigen(long2& val)
    {
        return Eigen::Map<Eigen::Matrix<long int, 2, 1>>(reinterpret_cast<long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long int, 2, 1>> as_eigen(const long2& val)
    {
        return Eigen::Map<const Eigen::Matrix<long int, 2, 1>>(
            reinterpret_cast<const long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long int, 3, 1>> as_eigen(long3& val)
    {
        return Eigen::Map<Eigen::Matrix<long int, 3, 1>>(reinterpret_cast<long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long int, 3, 1>> as_eigen(const long3& val)
    {
        return Eigen::Map<const Eigen::Matrix<long int, 3, 1>>(
            reinterpret_cast<const long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long int, 4, 1>> as_eigen(long4& val)
    {
        return Eigen::Map<Eigen::Matrix<long int, 4, 1>>(reinterpret_cast<long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long int, 4, 1>> as_eigen(const long4& val)
    {
        return Eigen::Map<const Eigen::Matrix<long int, 4, 1>>(
            reinterpret_cast<const long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long int, 2, 1>> as_eigen(ulong2& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long int, 2, 1>>(
            reinterpret_cast<unsigned long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long int, 2, 1>> as_eigen(const ulong2& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long int, 2, 1>>(
            reinterpret_cast<const unsigned long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long int, 3, 1>> as_eigen(ulong3& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long int, 3, 1>>(
            reinterpret_cast<unsigned long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long int, 3, 1>> as_eigen(const ulong3& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long int, 3, 1>>(
            reinterpret_cast<const unsigned long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long int, 4, 1>> as_eigen(ulong4& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long int, 4, 1>>(
            reinterpret_cast<unsigned long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long int, 4, 1>> as_eigen(const ulong4& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long int, 4, 1>>(
            reinterpret_cast<const unsigned long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long long int, 2, 1>> as_eigen(longlong2& val)
    {
        return Eigen::Map<Eigen::Matrix<long long int, 2, 1>>(
            reinterpret_cast<long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long long int, 2, 1>> as_eigen(const longlong2& val)
    {
        return Eigen::Map<const Eigen::Matrix<long long int, 2, 1>>(
            reinterpret_cast<const long long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long long int, 3, 1>> as_eigen(longlong3& val)
    {
        return Eigen::Map<Eigen::Matrix<long long int, 3, 1>>(
            reinterpret_cast<long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long long int, 3, 1>> as_eigen(const longlong3& val)
    {
        return Eigen::Map<const Eigen::Matrix<long long int, 3, 1>>(
            reinterpret_cast<const long long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<long long int, 4, 1>> as_eigen(longlong4& val)
    {
        return Eigen::Map<Eigen::Matrix<long long int, 4, 1>>(
            reinterpret_cast<long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<long long int, 4, 1>> as_eigen(const longlong4& val)
    {
        return Eigen::Map<const Eigen::Matrix<long long int, 4, 1>>(
            reinterpret_cast<const long long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long long int, 2, 1>> as_eigen(ulonglong2& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long long int, 2, 1>>(
            reinterpret_cast<unsigned long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long long int, 2, 1>> as_eigen(
        const ulonglong2& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long long int, 2, 1>>(
            reinterpret_cast<const unsigned long long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long long int, 3, 1>> as_eigen(ulonglong3& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long long int, 3, 1>>(
            reinterpret_cast<unsigned long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long long int, 3, 1>> as_eigen(
        const ulonglong3& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long long int, 3, 1>>(
            reinterpret_cast<const unsigned long long int*>(&val));
    }

    MUDA_INLINE MUDA_GENERIC Eigen::Map<Eigen::Matrix<unsigned long long int, 4, 1>> as_eigen(ulonglong4& val)
    {
        return Eigen::Map<Eigen::Matrix<unsigned long long int, 4, 1>>(
            reinterpret_cast<unsigned long long int*>(&val));
    }
    MUDA_INLINE MUDA_GENERIC Eigen::Map<const Eigen::Matrix<unsigned long long int, 4, 1>> as_eigen(
        const ulonglong4& val)
    {
        return Eigen::Map<const Eigen::Matrix<unsigned long long int, 4, 1>>(
            reinterpret_cast<const unsigned long long int*>(&val));
    }
}  // namespace eigen
}  // namespace muda