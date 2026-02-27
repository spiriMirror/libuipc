#pragma once
#include <muda/logger/logger_viewer.h>

namespace muda
{
// signed
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, char1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, char2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, char3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, char4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, short1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, short2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, short3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, short4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, int1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, int2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, int3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, int4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, long1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, long2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, long3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, long4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, longlong1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, longlong2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, longlong3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, longlong4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}

// unsigned

MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uchar1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uchar2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uchar3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uchar4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uint1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uint2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uint3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, uint4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulong1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulong2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulong3 val)
{
    return proxy << val.x << "," << val.y << ","
                 << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulong4 val)
{
    return proxy << val.x << "," << val.y << ","
                 << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulonglong1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulonglong2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulonglong3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, ulonglong4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
// float

MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, float1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, float2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, float3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, float4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, double1 val)
{
    return proxy << val.x;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, double2 val)
{
    return proxy << val.x << "," << val.y;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, double3 val)
{
    return proxy << val.x << "," << val.y << "," << val.z;
}
MUDA_INLINE MUDA_DEVICE LogProxy operator<<(LogProxy proxy, double4 val)
{
    return proxy << val.x << "," << val.y << "," << val.z << "," << val.w;
}
}  // namespace muda