#pragma once
#include <muda/launch/parallel_for.h>
#include <muda/ext/field/field_entry_view.h>
#include <muda/buffer/buffer_view.h>
#include <muda/ext/eigen/eigen_core_cxx20.h>
namespace muda
{
class FieldEntryLaunch : public LaunchBase<FieldEntryLaunch>
{
  public:
    MUDA_GENERIC FieldEntryLaunch(cudaStream_t stream = nullptr)
        : LaunchBase(stream)
    {
    }

    /**********************************************************************************************
    * 
    * EntryView <- EntryView
    * 
    **********************************************************************************************/
    template <typename T, FieldEntryLayout DstLayout, FieldEntryLayout SrcLayout, int M, int N>
    MUDA_HOST FieldEntryLaunch& copy(const FieldEntryViewT<false, T, DstLayout, M, N>& dst,
                                     const FieldEntryViewT<true, T, SrcLayout, M, N>& src);

    /**********************************************************************************************
    *   
    * EntryView <- Value
    *   
    * *********************************************************************************************/
    template <typename T, FieldEntryLayout DstLayout, int M, int N>
    MUDA_HOST FieldEntryLaunch& fill(
        const FieldEntryViewT<false, T, DstLayout, M, N>& dst,
        const typename FieldEntryViewT<true, T, DstLayout, M, N>::ValueT& value);

    /**********************************************************************************************
    *   
    * BufferView <- EntryView
    *   
    * *********************************************************************************************/
    template <typename T, FieldEntryLayout SrcLayout, int M, int N>
    MUDA_HOST FieldEntryLaunch& copy(
        BufferView<typename FieldEntryViewT<true, T, SrcLayout, M, N>::ValueT> dst,
        const FieldEntryViewT<true, T, SrcLayout, M, N>& src);

    /**********************************************************************************************
    *   
    * EntryView <- BufferView
    *   
    * *********************************************************************************************/
    template <typename T, FieldEntryLayout DstLayout, int M, int N>
    MUDA_HOST FieldEntryLaunch& copy(
        const FieldEntryViewT<false, T, DstLayout, M, N>& dst,
        CBufferView<typename FieldEntryViewT<true, T, DstLayout, M, N>::ValueT> src);
};
}  // namespace muda

#include "details/field_entry_launch.inl"