#pragma once
#include <inttypes.h>

namespace muda::spgrid
{
static uint64_t expand_bits(uint32_t v)
{
    uint64_t x = v;
    x          = (x | (x << 32)) & 0x1f00000000ffff;
    x          = (x | (x << 16)) & 0x1f0000ff0000ff;
    x          = (x | (x << 8)) & 0x100f00f00f00f00f;
    x          = (x | (x << 4)) & 0x10c30c30c30c30c3;
    x          = (x | (x << 2)) & 0x1249249249249249;
    return x;
}

static uint32_t compact_bits(uint64_t x)
{
    x = x & 0x1249249249249249;
    x = (x | (x >> 2)) & 0x10c30c30c30c30c3;
    x = (x | (x >> 4)) & 0x100f00f00f00f00f;
    x = (x | (x >> 8)) & 0x1f0000ff0000ff;
    x = (x | (x >> 16)) & 0x1f00000000ffff;
    x = (x | (x >> 32)) & 0x1fffff;
    return (uint32_t)x;
}

static uint64_t encode_morton(int x, int y, int z)
{
    return expand_bits(x) | (expand_bits(y) << 1) | (expand_bits(z) << 2);
}

static void decode_morton(uint64_t morton, int* x, int* y, int* z)
{
    *x = compact_bits(morton);
    *y = compact_bits(morton >> 1);
    *z = compact_bits(morton >> 2);
}

static uint64_t packed_add_morton(uint64_t data, int x, int y, int z)
{
    // Decode the original Morton code
    int orig_x, orig_y, orig_z;
    decode_morton(data, &orig_x, &orig_y, &orig_z);

    // Apply the offset
    int new_x = orig_x + x;
    int new_y = orig_y + y;
    int new_z = orig_z + z;

    // Boundary check (assuming non-negative coordinate range)
    if(new_x < 0)
        new_x = 0;
    if(new_y < 0)
        new_y = 0;
    if(new_z < 0)
        new_z = 0;

    // Re-encode as Morton code
    return encode_morton(new_x, new_y, new_z);
}

// Apply an offset to a linear encoding and return the new linear encoding
// Assume the linear encoding is based on some 3D space partitioning rule (e.g., voxel grid)
// Here, assume it is within a fixed-size 3D grid
template <size_t nx = 4, size_t ny = 4, size_t nz = 4>
static uint64_t packed_add_linear(int id, int x, int y, int z)
{
    constexpr int GRID_SIZE_X = nx;
    constexpr int GRID_SIZE_Y = ny;
    constexpr int GRID_SIZE_Z = nz;

    // Decode the 3D coordinates from the linear ID
    int orig_z    = id / (GRID_SIZE_X * GRID_SIZE_Y);
    int remaining = id % (GRID_SIZE_X * GRID_SIZE_Y);
    int orig_y    = remaining / GRID_SIZE_X;
    int orig_x    = remaining % GRID_SIZE_X;

    // Apply the offset
    int new_x = orig_x + x;
    int new_y = orig_y + y;
    int new_z = orig_z + z;

    // Boundary check and wrap-around handling
    new_x = ((new_x % GRID_SIZE_X) + GRID_SIZE_X) % GRID_SIZE_X;
    new_y = ((new_y % GRID_SIZE_Y) + GRID_SIZE_Y) % GRID_SIZE_Y;
    new_z = ((new_z % GRID_SIZE_Z) + GRID_SIZE_Z) % GRID_SIZE_Z;

    // Re-encode as a linear ID
    return new_z * (GRID_SIZE_X * GRID_SIZE_Y) + new_y * GRID_SIZE_X + new_x;
}

int encode_linear(int x, int y, int z, int nx, int ny, int nz);
}  // namespace muda::spgrid
