#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/backend/type_define.h>
#include <uipc/common/type_define.h>
namespace uipc::backend
{
class UIPC_CORE_API BufferView
{
  private:
    static constexpr SizeT INVALID = ~0;

  public:
    BufferView() = default;
    BufferView(HandleT          handle,
               SizeT            element_offset,
               SizeT            element_count,
               SizeT            element_size,
               SizeT            element_stride,
               std::string_view backend_name) noexcept;

    HandleT          handle() const noexcept;
    SizeT            offset() const noexcept;
    SizeT            size() const noexcept;
    SizeT            element_size() const noexcept;
    SizeT            element_stride() const noexcept;
    SizeT            size_in_bytes() const noexcept;
    std::string_view backend() const noexcept;
    operator bool() const noexcept;

    BufferView subview(SizeT offset, SizeT element_count) const noexcept;

  private:
    HandleT m_handle = 0;
    SizeT   m_offset = INVALID;
    SizeT   m_size   = 0;

    SizeT            m_element_size   = INVALID;
    SizeT            m_element_stride = INVALID;
    std::string_view m_backend;
};
}  // namespace uipc::backend
