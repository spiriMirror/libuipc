#pragma once
#include <muda/tools/version.h>
#include <thrust/device_allocator.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <vector>

#include <muda/muda_def.h>
#include <muda/buffer/buffer_view.h>
#include <muda/viewer/dense.h>

namespace muda
{
namespace details
{
    template <typename T, typename Alloc>
    using vector_base = thrust::detail::vector_base<T, Alloc>;
}

template <typename T>
class DeviceVector : public thrust::device_vector<T, thrust::device_allocator<T>>
{
  public:
    using Base = thrust::device_vector<T, thrust::device_allocator<T>>;
    using Base::Base;
    using Base::operator=;

    auto view() MUDA_NOEXCEPT { return BufferView<T>{raw_ptr(), Base::size()}; }

    auto view() const MUDA_NOEXCEPT
    {
        return CBufferView<T>{raw_ptr(), Base::size()};
    }

    operator BufferView<T>() const MUDA_NOEXCEPT { return view(); }
    operator CBufferView<T>() const MUDA_NOEXCEPT { return view(); }

    DeviceVector& operator=(CBufferView<T> v)
    {
        this->resize(v.size());
        view().copy_from(v);
        return *this;
    }

    void copy_to(std::vector<T>& v) const
    {
        v.resize(this->size());
        view().copy_to(v.data());
    }

    auto viewer() MUDA_NOEXCEPT
    {
        return Dense1D<T>(raw_ptr(), static_cast<int>(this->size()));
    }

    auto cviewer() const MUDA_NOEXCEPT
    {
        return CDense1D<T>(raw_ptr(), static_cast<int>(this->size()));
    }

  private:
    T*       raw_ptr() { return thrust::raw_pointer_cast(Base::data()); }
    const T* raw_ptr() const { return thrust::raw_pointer_cast(Base::data()); }
};

template <typename T>
class HostVector : public thrust::host_vector<T, std::allocator<T>>
{
  public:
    using thrust::host_vector<T, std::allocator<T>>::host_vector;
    using thrust::host_vector<T, std::allocator<T>>::operator=;
};
}  // namespace muda


//namespace muda
//{
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense(DeviceVector<T>& v) MUDA_NOEXCEPT
//{
//    return make_dense(v.view());
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense(const DeviceVector<T>& v) MUDA_NOEXCEPT
//{
//    return make_cdense(v.view());
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_viewer(DeviceVector<T>& v) MUDA_NOEXCEPT
//{
//    return make_viewer(v.view());
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cviewer(const DeviceVector<T>& v) MUDA_NOEXCEPT
//{
//    return make_cviewer(v.view());
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense2D(DeviceVector<T>& v, int dimy) MUDA_NOEXCEPT
//{
//    return make_dense2D(v.view(), dimy);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense2D(const DeviceVector<T>& v, int dimy) MUDA_NOEXCEPT
//{
//    return make_cdense2D(v.view(), dimy);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense2D(DeviceVector<T>& v, int dimx, int dimy) MUDA_NOEXCEPT
//{
//    return make_dense2D(v.view(), dimx, dimy);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense2D(const DeviceVector<T>& v, int dimx, int dimy) MUDA_NOEXCEPT
//{
//    return make_cdense2D(v.view(), dimx, dimy);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense2D(DeviceVector<T>& v, const int2& dim) MUDA_NOEXCEPT
//{
//    return make_dense2D(v.view(), dim.x, dim.y);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense2D(const DeviceVector<T>& v, const int2& dim) MUDA_NOEXCEPT
//{
//    return make_cdense2D(v.view(), dim.x, dim.y);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense3D(DeviceVector<T>& v, int dimy, int dimz) MUDA_NOEXCEPT
//{
//    return make_dense3D(v.view(), dimy, dimz);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense3D(const DeviceVector<T>& v, int dimy, int dimz) MUDA_NOEXCEPT
//{
//    return make_cdense3D(v.view(), dimy, dimz);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense3D(DeviceVector<T>& v, const int2& dimyz) MUDA_NOEXCEPT
//{
//    return make_dense3D(v.view(), dimyz.x, dimyz.y);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense3D(const DeviceVector<T>& v, const int2& dimyz) MUDA_NOEXCEPT
//{
//    return make_cdense3D(v.view(), dimyz.x, dimyz.y);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense3D(DeviceVector<T>& v, int dimx, int dimy, int dimz) MUDA_NOEXCEPT
//{
//    return make_dense3D(v.view(), dimx, dimy, dimz);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense3D(const DeviceVector<T>& v, int dimx, int dimy, int dimz) MUDA_NOEXCEPT
//{
//    return make_cdense3D(v.view(), dimx, dimy, dimz);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_dense3D(DeviceVector<T>& v, const int3& dim) MUDA_NOEXCEPT
//{
//    return make_dense3D(v.view(), dim.x, dim.y, dim.z);
//}
//template <typename T>
//MUDA_INLINE MUDA_HOST auto make_cdense3D(const DeviceVector<T>& v, const int3& dim) MUDA_NOEXCEPT
//{
//    return make_cdense3D(v.view(), dim.x, dim.y, dim.z);
//}
//}  // namespace muda