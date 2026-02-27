#pragma once
#include <cinttypes>
namespace muda
{
enum class FieldEntryLayout : uint32_t
{
    None,
    // Array of Struct
    AoS,
    // Struct of Array
    SoA,
    // Array of Struct of Array
    // The innermost Array must be fixed size
    // e.g. size = 32 (warp size)
    AoSoA
};
}