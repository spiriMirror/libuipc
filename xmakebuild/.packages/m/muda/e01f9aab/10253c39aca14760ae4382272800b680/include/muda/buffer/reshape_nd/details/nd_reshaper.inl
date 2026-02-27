#include <list>
#include <array>
#include <bitset>
#include <muda/buffer/device_buffer.h>
#include <muda/buffer/device_buffer_2d.h>
#include <muda/buffer/device_buffer_3d.h>
#include <muda/buffer/reshape_nd/reserve.h>
#include <muda/buffer/reshape_nd/masked_compare.h>
#include <muda/buffer/reshape_nd/masked_swap.h>

namespace muda::details::buffer
{
template <typename BufferView>
class CopyConstructInfo
{
  public:
    BufferView dst;
    BufferView src;
};

template <typename BufferView>
class ConstructInfo
{
  public:
    BufferView dst;
};

template <typename BufferView>
class DestructInfo
{
  public:
    BufferView dst;
};

template <typename T, size_t N>
using Array = std::array<T, N>;
template <size_t N>
using Offset = std::array<size_t, N>;

template <typename F, size_t N>
void for_all_cell(const Array<Array<size_t, 3>, N>& offsets, F&& f)
{
    using namespace std;
    constexpr auto total = 1 << N;
#pragma unroll
    for(size_t index = 0; index < total; ++index)
    {
        bitset<N> bits{index};
        Offset<N> begin, end;
        bitset<N> mask;
#pragma unroll
        for(size_t c = 0; c < N; ++c)  // c : component
        {
            auto i = bits[c];
            mask.set(c, i != 0);
            begin[c] = offsets[c][i];
            end[c]   = offsets[c][i + 1];
        }
        f(mask, begin, end);
    }
}
}  // namespace muda::details::buffer


namespace muda
{
template <typename T, typename FConstruct>
void NDReshaper::resize(int              grid_dim,
                        int              block_dim,
                        cudaStream_t     stream,
                        DeviceBuffer<T>& buffer,
                        size_t           new_size,
                        FConstruct&&     fct)
{
    using namespace details::buffer;

    auto& m_data     = buffer.m_data;
    auto& m_size     = buffer.m_size;
    auto& m_capacity = buffer.m_capacity;

    if(new_size == m_size)
        return;

    auto          old_size   = m_size;
    BufferView<T> old_buffer = buffer.view();
    BufferView<T> new_buffer;

    if(new_size < m_size)
    {
        // destruct the old memory
        auto to_destruct = buffer.view(new_size, old_size - new_size);
        kernel_destruct<T>(grid_dim, block_dim, stream, to_destruct);
        m_size     = new_size;
        new_buffer = old_buffer;
        return;
    }

    if(new_size <= m_capacity)
    {
        // construct the new memory
        BufferView<T> to_construct = BufferView<T>{m_data + old_size, new_size - old_size};
        //auto to_construct = old_buffer.subview(old_size, new_size - old_size);
        fct(to_construct);
        m_size     = new_size;
        new_buffer = old_buffer;
        return;
    }
    else
    {
        new_buffer = reserve_1d<T>(stream, new_size);

        if(m_data)
        {
            auto to_copy_construct = new_buffer.subview(0, old_size);
            // copy construct on new memory
            kernel_copy_construct<T>(grid_dim, block_dim, stream, to_copy_construct, old_buffer);
        }

        // construct the rest new memory
        {
            BufferView<T> to_construct = new_buffer.subview(old_size);
            fct(to_construct);
        }

        if(m_data)
        {
            // destruct the old memory
            kernel_destruct<T>(grid_dim, block_dim, stream, buffer.view());
            // free the old memory
            Memory(stream).free(m_data);
        }


        m_data     = new_buffer.origin_data();
        m_size     = new_size;
        m_capacity = new_size;
        return;
    }
}

template <typename T>
MUDA_HOST void NDReshaper::shrink_to_fit(int              grid_dim,
                                         int              block_dim,
                                         cudaStream_t     stream,
                                         DeviceBuffer<T>& buffer)
{
    using namespace details::buffer;
    auto& m_data     = buffer.m_data;
    auto& m_size     = buffer.m_size;
    auto& m_capacity = buffer.m_capacity;

    auto          old_buffer = buffer.view();
    BufferView<T> new_buffer;

    if(m_size == m_capacity)
        return;


    if(m_size > 0)
    {
        // alloc new buffer
        new_buffer = reserve_1d<T>(stream, m_size);
        // copy construct on the new buffer
        kernel_copy_construct<T>(grid_dim, block_dim, stream, new_buffer, old_buffer);
    }

    if(old_buffer.origin_data())
    {
        // destruct the old buffer
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        // free the old buffer
        Memory(stream).free(m_data);
    }

    m_data     = new_buffer.origin_data();
    m_capacity = m_size;
}

template <typename T>
MUDA_HOST void NDReshaper::reserve(int              grid_dim,
                                   int              block_dim,
                                   cudaStream_t     stream,
                                   DeviceBuffer<T>& buffer,
                                   size_t           new_capacity)
{
    using namespace details::buffer;

    auto& m_data     = buffer.m_data;
    auto& m_size     = buffer.m_size;
    auto& m_capacity = buffer.m_capacity;

    auto old_buffer = buffer.view();

    if(new_capacity <= buffer.capacity())
        return;

    BufferView<T> new_buffer = reserve_1d<T>(stream, new_capacity);
    // copy construct
    auto to_copy_construct = new_buffer.subview(0, old_buffer.size());
    kernel_copy_construct<T>(grid_dim, block_dim, stream, to_copy_construct, old_buffer);

    if(old_buffer.origin_data())
    {
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        Memory(stream).free(old_buffer.origin_data());
    }

    m_data     = new_buffer.origin_data();
    m_capacity = new_buffer.size();
}

template <typename T, typename FConstruct>
MUDA_HOST void NDReshaper::resize(int                grid_dim,
                                  int                block_dim,
                                  cudaStream_t       stream,
                                  DeviceBuffer2D<T>& buffer,
                                  Extent2D           new_extent,
                                  FConstruct&&       fct)
{
    using namespace details::buffer;

    auto& m_data        = buffer.m_data;
    auto& m_pitch_bytes = buffer.m_pitch_bytes;
    auto& m_extent      = buffer.m_extent;
    auto& m_capacity    = buffer.m_capacity;

    if(new_extent == m_extent)
        return;

    auto old_extent = m_extent;

    std::list<CopyConstructInfo<Buffer2DView<T>>> copy_construct_infos;
    std::list<ConstructInfo<Buffer2DView<T>>>     construct_infos;
    std::list<DestructInfo<Buffer2DView<T>>>      destruct_infos;

    Buffer2DView<T> old_buffer = buffer.view();
    Buffer2DView<T> new_buffer;
    if(new_extent <= m_capacity)
    {
        // all dimensions are bigger than the new extent
        m_extent   = new_extent;
        new_buffer = old_buffer;
    }
    else
    {
        // at least one dimension is smaller than the new extent
        // so we need to allocate a new buffer (m_capacity)
        // which is bigger than the new_extent in all dimensions
        auto new_capacity = max(new_extent, m_capacity);
        new_buffer        = reserve_2d<T>(stream, new_capacity);

        m_data        = new_buffer.origin_data();
        m_pitch_bytes = new_buffer.pitch_bytes();
        m_extent      = new_extent;
        m_capacity    = new_capacity;
    }

    constexpr size_t           N = 2;
    Array<Array<size_t, 3>, N> offsets;
    //tex:
    // $$
    // \begin{bmatrix}
    // 0 & w_0 & w_1 \\
    // 0 & h_0 & h_1
    // \end{bmatrix}
    // $$
    offsets[0]     = {0ull, old_extent.width(), new_extent.width()};
    offsets[1]     = {0ull, old_extent.height(), new_extent.height()};
    bool need_copy = (new_buffer.data(0) != nullptr);
    for_all_cell(offsets,
                 [&](std::bitset<N> mask, Offset<N>& begin, Offset<N>& end)
                 {
                     bool copy_construct = !mask.any();
                     if(copy_construct)
                     {
                         // all DOF are fixed

                         if(new_buffer.origin_data() != old_buffer.origin_data())
                         {
                             // if new_buffer is allocated, we need to copy the old data
                             Offset2D offset_begin{begin[1], begin[0]};
                             Offset2D offset_end{end[1], end[0]};
                             Extent2D extent = as_extent(offset_end - offset_begin);

                             CopyConstructInfo<Buffer2DView<T>> info;
                             info.dst = new_buffer.subview(offset_begin, extent);
                             info.src = old_buffer.subview(offset_begin, extent);
                             copy_construct_infos.push_back(std::move(info));
                         }
                         else
                         {
                             // we don't need to copy the old data
                         }
                         return;
                     }
                     else
                     {
                         // some DOF are fixed
                         bool construct = less(mask, begin, end);
                         if(construct)
                         {
                             Offset2D offset_begin{begin[1], begin[0]};
                             Offset2D offset_end{end[1], end[0]};
                             Extent2D extent = as_extent(offset_end - offset_begin);
                             ConstructInfo<Buffer2DView<T>> info;
                             info.dst = new_buffer.subview(offset_begin, extent);
                             construct_infos.emplace_back(std::move(info));
                             return;
                         }
                         bool destruct = less(mask, end, begin);
                         if(destruct)
                         {
                             swap(mask, begin, end);
                             Offset2D offset_begin{begin[1], begin[0]};
                             Offset2D offset_end{end[1], end[0]};
                             Extent2D extent = as_extent(offset_end - offset_begin);
                             DestructInfo<Buffer2DView<T>> info;
                             info.dst = old_buffer.subview(offset_begin, extent);

                             destruct_infos.emplace_back(std::move(info));
                             return;
                         }
                     }
                     // else we need to do nothing
                 });

    // destruct
    for(auto& info : destruct_infos)
        kernel_destruct<T>(grid_dim, block_dim, stream, info.dst);
    // construct
    for(auto& info : construct_infos)
        fct(info.dst);
    // copy construct
    for(auto& info : copy_construct_infos)
        kernel_copy_construct<T>(grid_dim, block_dim, stream, info.dst, info.src);


    // if the new buffer was allocated, deallocate the old one
    if(new_buffer.origin_data() != old_buffer.origin_data())
    {
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        Memory(stream).free(old_buffer.origin_data());
    }
    return;
}

template <typename T>
MUDA_HOST void NDReshaper::shrink_to_fit(int                grid_dim,
                                         int                block_dim,
                                         cudaStream_t       stream,
                                         DeviceBuffer2D<T>& buffer)
{
    using namespace details::buffer;
    auto& m_data        = buffer.m_data;
    auto& m_pitch_bytes = buffer.m_pitch_bytes;
    auto& m_extent      = buffer.m_extent;
    auto& m_capacity    = buffer.m_capacity;

    auto            old_buffer = buffer.view();
    Buffer2DView<T> new_buffer;

    if(m_extent == m_capacity)
        return;


    if(!(m_extent == Extent2D::Zero()))
    {
        // alloc new buffer
        new_buffer = reserve_2d<T>(stream, m_extent);

        // copy construct on new buffer
        kernel_copy_construct<T>(grid_dim, block_dim, stream, new_buffer, old_buffer);

        m_pitch_bytes = new_buffer.pitch_bytes();
    }

    if(old_buffer.origin_data())
    {
        // destruct on old buffer
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);

        // free old buffer
        Memory(stream).free(old_buffer.origin_data());
    }

    m_data     = new_buffer.origin_data();
    m_capacity = m_extent;
}

template <typename T>
MUDA_HOST void NDReshaper::reserve(int                grid_dim,
                                   int                block_dim,
                                   cudaStream_t       stream,
                                   DeviceBuffer2D<T>& buffer,
                                   Extent2D           new_capacity)
{
    using namespace details::buffer;

    auto& m_data        = buffer.m_data;
    auto& m_pitch_bytes = buffer.m_pitch_bytes;
    auto& m_extent      = buffer.m_extent;
    auto& m_capacity    = buffer.m_capacity;

    auto old_buffer = buffer.view();

    if(new_capacity <= m_capacity)
        return;

    new_capacity = max(new_capacity, m_capacity);

    Buffer2DView<T> new_buffer = reserve_2d<T>(stream, new_capacity);
    // copy construct
    auto to_copy_construct = new_buffer.subview(Offset2D::Zero(), m_extent);
    kernel_copy_construct<T>(grid_dim, block_dim, stream, to_copy_construct, old_buffer);

    if(old_buffer.origin_data())
    {
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        Memory(stream).free(old_buffer.origin_data());
    }

    m_data        = new_buffer.origin_data();
    m_pitch_bytes = new_buffer.pitch_bytes();
    m_capacity    = new_capacity;
}

template <typename T, typename FConstruct>
MUDA_HOST void NDReshaper::resize(int                grid_dim,
                                  int                block_dim,
                                  cudaStream_t       stream,
                                  DeviceBuffer3D<T>& buffer,
                                  Extent3D           new_extent,
                                  FConstruct&&       fct)
{
    using namespace details::buffer;

    auto& m_data             = buffer.m_data;
    auto& m_pitch_bytes      = buffer.m_pitch_bytes;
    auto& m_pitch_bytes_area = buffer.m_pitch_bytes_area;
    auto& m_extent           = buffer.m_extent;
    auto& m_capacity         = buffer.m_capacity;

    if(new_extent == m_extent)
        return;

    auto old_extent = m_extent;

    std::list<CopyConstructInfo<Buffer3DView<T>>> copy_construct_infos;
    std::list<ConstructInfo<Buffer3DView<T>>>     construct_infos;
    std::list<DestructInfo<Buffer3DView<T>>>      destruct_infos;

    Buffer3DView<T> old_buffer = buffer.view();
    Buffer3DView<T> new_buffer;

    if(new_extent <= m_capacity)
    {
        // all dimensions are bigger than the new extent
        m_extent   = new_extent;
        new_buffer = old_buffer;
    }
    else
    {
        // at least one dimension is smaller than the new extent
        // so we need to allocate a new buffer (m_capacity)
        // which is bigger than the new_extent in all dimensions
        auto new_capacity = max(new_extent, m_capacity);
        new_buffer        = reserve_3d<T>(stream, new_capacity);

        m_data             = new_buffer.origin_data();
        m_pitch_bytes      = new_buffer.pitch_bytes();
        m_pitch_bytes_area = new_buffer.pitch_bytes_area();
        m_extent           = new_extent;
        m_capacity         = new_capacity;
    }

    constexpr size_t           N = 3;
    Array<Array<size_t, 3>, N> offsets;
    //tex:
    // $$
    // \begin{bmatrix}
    // 0 & w_0 & w_1 \\
    // 0 & h_0 & h_1 \\
    // 0 & d_0 & d_1
    // \end{bmatrix}
    // $$
    offsets[0]     = {0ull, old_extent.width(), new_extent.width()};
    offsets[1]     = {0ull, old_extent.height(), new_extent.height()};
    offsets[2]     = {0ull, old_extent.depth(), new_extent.depth()};
    bool need_copy = (new_buffer.data(0) != nullptr);
    for_all_cell(
        offsets,
        [&](std::bitset<N> mask, Offset<N>& begin, Offset<N>& end)
        {
            bool copy_construct = !mask.any();
            if(copy_construct)
            {
                // all DOF are fixed
                if(new_buffer.origin_data() != old_buffer.origin_data())
                {
                    // if new_buffer is allocated, we need to copy the old data
                    Offset3D offset_begin{begin[2], begin[1], begin[0]};
                    Offset3D offset_end{end[2], end[1], end[0]};
                    Extent3D extent = as_extent(offset_end - offset_begin);

                    CopyConstructInfo<Buffer3DView<T>> info;
                    info.dst = new_buffer.subview(offset_begin, extent);
                    info.src = old_buffer.subview(offset_begin, extent);
                    copy_construct_infos.emplace_back(info);
                }
                else
                {
                    // we don't need to copy the old data
                }
                return;
            }
            else
            {
                // some DOF are fixed
                bool construct = less(mask, begin, end);
                if(construct)
                {
                    Offset3D offset_begin{begin[2], begin[1], begin[0]};
                    Offset3D offset_end{end[2], end[1], end[0]};
                    Extent3D extent = as_extent(offset_end - offset_begin);

                    ConstructInfo<Buffer3DView<T>> info;
                    info.dst = new_buffer.subview(offset_begin, extent);
                    construct_infos.emplace_back(std::move(info));
                    return;
                }
                bool destruct = less(mask, end, begin);
                if(destruct)
                {
                    swap(mask, begin, end);
                    Offset3D offset_begin{begin[2], begin[1], begin[0]};
                    Offset3D offset_end{end[2], end[1], end[0]};
                    Extent3D extent = as_extent(offset_end - offset_begin);

                    DestructInfo<Buffer3DView<T>> info;
                    info.dst = old_buffer.subview(offset_begin, extent);

                    destruct_infos.emplace_back(std::move(info));
                    return;
                }
            }
            // else we need to do nothing
        });

    // destruct
    for(auto& info : destruct_infos)
        kernel_destruct<T>(grid_dim, block_dim, stream, info.dst);
    // construct
    for(auto& info : construct_infos)
        fct(info.dst);
    // copy construct
    for(auto& info : copy_construct_infos)
        kernel_copy_construct<T>(grid_dim, block_dim, stream, info.dst, info.src);

    // if the new buffer was allocated, deallocate the old one
    if(new_buffer.origin_data() != old_buffer.origin_data())
    {
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        Memory(stream).free(old_buffer.origin_data());
    }


    return;
}

template <typename T>
MUDA_HOST void NDReshaper::shrink_to_fit(int                grid_dim,
                                         int                block_dim,
                                         cudaStream_t       stream,
                                         DeviceBuffer3D<T>& buffer)
{
    using namespace details::buffer;
    auto& m_data             = buffer.m_data;
    auto& m_pitch_bytes      = buffer.m_pitch_bytes;
    auto& m_pitch_bytes_area = buffer.m_pitch_bytes_area;
    auto& m_extent           = buffer.m_extent;
    auto& m_capacity         = buffer.m_capacity;

    auto            old_buffer = buffer.view();
    Buffer3DView<T> new_buffer;

    if(m_extent == m_capacity)
        return;

    if(!(m_extent == Extent3D::Zero()))
    {
        // alloc new buffer
        new_buffer = reserve_3d<T>(stream, m_extent);

        // copy construct on new buffer
        kernel_copy_construct<T>(grid_dim, block_dim, stream, new_buffer, old_buffer);

        m_pitch_bytes      = new_buffer.pitch_bytes();
        m_pitch_bytes_area = new_buffer.pitch_bytes_area();
    }

    if(old_buffer.origin_data())
    {
        // destruct on old buffer
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);

        // free old buffer
        Memory(stream).free(old_buffer.origin_data());
    }

    m_data     = new_buffer.origin_data();
    m_capacity = m_extent;
}

template <typename T>
MUDA_HOST void NDReshaper::reserve(int                grid_dim,
                                   int                block_dim,
                                   cudaStream_t       stream,
                                   DeviceBuffer3D<T>& buffer,
                                   Extent3D           new_capacity)
{
    using namespace details::buffer;

    auto& m_data             = buffer.m_data;
    auto& m_pitch_bytes      = buffer.m_pitch_bytes;
    auto& m_pitch_bytes_area = buffer.m_pitch_bytes_area;
    auto& m_extent           = buffer.m_extent;
    auto& m_capacity         = buffer.m_capacity;

    auto old_buffer = buffer.view();

    if(new_capacity <= m_capacity)
        return;

    new_capacity = max(new_capacity, m_capacity);

    Buffer3DView<T> new_buffer = reserve_3d<T>(stream, new_capacity);
    // copy construct
    auto to_copy_construct = new_buffer.subview(Offset3D::Zero(), m_extent);
    kernel_copy_construct<T>(grid_dim, block_dim, stream, to_copy_construct, old_buffer);

    if(old_buffer.origin_data())
    {
        kernel_destruct<T>(grid_dim, block_dim, stream, old_buffer);
        Memory(stream).free(old_buffer.origin_data());
    }

    m_data             = new_buffer.origin_data();
    m_pitch_bytes      = new_buffer.pitch_bytes();
    m_pitch_bytes_area = new_buffer.pitch_bytes_area();
    m_capacity         = new_capacity;
}
}  // namespace muda