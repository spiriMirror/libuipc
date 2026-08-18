#include <muda/ext/field/field_entry_launch.h>

namespace muda
{
template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
template <FieldEntryLayout SrcLayout>
MUDA_HOST void FieldEntryViewT<IsConst, T, Layout, M, N>::copy_from(
    const FieldEntryViewT<true, T, SrcLayout, M, N>& src) const MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot copy to const view");

    FieldEntryLaunch()  //
        .copy(*this, src)
        .wait();
}

template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
MUDA_HOST void FieldEntryViewT<IsConst, T, Layout, M, N>::copy_from(const CBufferView<ValueT>& src) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot copy to const view");

    FieldEntryLaunch()  //
        .copy(*this, src)
        .wait();
}

template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
MUDA_HOST void FieldEntryViewT<IsConst, T, Layout, M, N>::copy_to(const BufferView<ValueT>& dst) const
{
    FieldEntryLaunch()  //
        .copy(dst, this->as_const())
        .wait();
}

template <bool IsConst, typename T, FieldEntryLayout Layout, int M, int N>
MUDA_HOST void FieldEntryViewT<IsConst, T, Layout, M, N>::fill(const ValueT& value) const
    MUDA_REQUIRES(!IsConst)
{
    static_assert(!IsConst, "Cannot fill const view");

    FieldEntryLaunch()  //
        .fill(*this, value)
        .wait();
}
}  // namespace muda