#pragma once
#include <cinttypes>
#include <muda/muda_def.h>
#include <muda/tools/debug_log.h>
namespace muda
{
enum class FieldEntryLayout
{
    None,
    // Array of Struct
    AoS,
    // Struct of Array
    SoA,
    // Array of Struct of Array
    // The innermost Array must be fixed size
    // e.g. size = 32 (warp size)
    AoSoA,

    // the layout is not known at compile time
    RuntimeLayout,
};

class FieldEntryLayoutInfo
{
    using Layout = FieldEntryLayout;

  public:
    MUDA_GENERIC auto layout() const MUDA_NOEXCEPT { return m_layout; }
    MUDA_GENERIC auto innermost_array_size() const MUDA_NOEXCEPT
    {
        return m_innermost_array_size;
    }

    MUDA_GENERIC FieldEntryLayoutInfo(Layout layout, uint32_t innermost_array_size = 32) MUDA_NOEXCEPT
        : m_layout(layout),
          m_innermost_array_size(layout == Layout::AoSoA ? innermost_array_size : 0)
    {
        MUDA_ASSERT(layout != Layout::RuntimeLayout,
                    "RuntimeLayout is not allowed to use when constructing FieldEntryLayoutInfo, because it's meaningless."
                    "RuntimeLayout is only used in template argument.");

        MUDA_ASSERT((innermost_array_size & (innermost_array_size - 1)) == 0,
                    "innermost_array_size must be power of 2");
    }

    MUDA_GENERIC FieldEntryLayoutInfo() MUDA_NOEXCEPT {}

  private:
    Layout   m_layout               = Layout::AoSoA;
    uint32_t m_innermost_array_size = 32;
};
}  // namespace muda