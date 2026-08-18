#pragma once
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include <muda/muda_def.h>
#include <muda/tools/debug_log.h>
#include <muda/muda_config.h>
#include <muda/assert.h>
#include <muda/tools/launch_info_cache.h>
#include <muda/tools/fuzzy.h>
#include <muda/type_traits/type_modifier.h>

namespace muda
{
template <bool IsConst_ = false>
class ViewerBase
{
  public:
    constexpr static bool IsConst    = IsConst_;
    constexpr static bool IsNonConst = !IsConst_;

  protected:
    template <typename T>
    using auto_const_t = std::conditional_t<IsConst, const T, T>;
    template <typename T>
    using non_const_enable_t = std::enable_if_t<IsNonConst, T>;

  private:
    // friend class details::ViewerBaseAccessor;

#if MUDA_CHECK_ON
    details::StringPointer m_viewer_name;
    details::StringPointer m_kernel_name;
    details::StringPointer m_kernel_file;
    int                    m_kernel_line = -1;
#else
    int m_dummy = 0;  // a dummy member to avoid empty class
#endif
  public:
    MUDA_GENERIC ViewerBase() MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
#ifndef __CUDA_ARCH__
            m_kernel_name = details::LaunchInfoCache::current_kernel_name();
            m_kernel_file = details::LaunchInfoCache::current_kernel_file();
            m_kernel_line = details::LaunchInfoCache::current_kernel_line();
#endif
#endif
    }

    MUDA_GENERIC const char* name() const MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        auto n = m_viewer_name.auto_select();
        if(n && *n != '\0')
            return n;
#endif
        return "~";
    }

    MUDA_GENERIC const char* kernel_name() const MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        auto n = m_kernel_name.auto_select();
        if(n && *n != '\0')
            return n;
#endif
        return "~";
    }

    MUDA_GENERIC const char* kernel_file() const MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        auto n = m_kernel_file.auto_select();
        if(n && *n != '\0')
            return n;
#endif
        return "~";
    }

    MUDA_GENERIC int kernel_line() const MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        return m_kernel_line;
#endif
        return -1;
    }

    MUDA_INLINE MUDA_GENERIC void copy_label(const ViewerBase& other) MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        m_viewer_name = other.m_viewer_name;
        m_kernel_name = other.m_kernel_name;
        m_kernel_file = other.m_kernel_file;
        m_kernel_line = other.m_kernel_line;
#endif
    }

    // default copy / move
    ViewerBase(const ViewerBase&)            = default;
    ViewerBase(ViewerBase&&)                 = default;
    ViewerBase& operator=(const ViewerBase&) = default;
    ViewerBase& operator=(ViewerBase&&)      = default;

  protected:
    MUDA_INLINE MUDA_HOST void name(const char* n) MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        m_viewer_name = details::LaunchInfoCache::view_name(n);
#endif
    }

    MUDA_INLINE MUDA_GENERIC void name(details::StringPointer pointer) MUDA_NOEXCEPT
    {
#if MUDA_CHECK_ON
        m_viewer_name = pointer;
#endif
    }
};

#define MUDA_VIEWER_COMMON_NAME(viewer_name)                                   \
  public:                                                                      \
    using this_type = viewer_name;                                             \
                                                                               \
    MUDA_INLINE MUDA_HOST this_type& name(const char* n) noexcept              \
    {                                                                          \
        ::muda::ViewerBase<viewer_name::IsConst>::name(n);                     \
        return *this;                                                          \
    }                                                                          \
                                                                               \
    MUDA_INLINE MUDA_GENERIC const char* name() const noexcept                 \
    {                                                                          \
        return ::muda::ViewerBase<viewer_name::IsConst>::name();               \
    }                                                                          \
                                                                               \
  private:
}  // namespace muda