#pragma once
#include <uipc/common/type_define.h>
#include <uipc/backend/buffer_view.h>
#include <functional>

namespace uipc::backend
{
class UIPC_CORE_API Buffer
{
  public:
    Buffer(std::function<void(SizeT)>  resize_func,
           std::function<BufferView()> get_buffer_view_func);

    void       resize(SizeT size);
    BufferView view() const;

  private:
    std::function<void(SizeT)>  m_resize_func;
    std::function<BufferView()> m_get_buffer_view_func;
};
}  // namespace uipc::backend