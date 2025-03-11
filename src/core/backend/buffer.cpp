#include <uipc/backend/buffer.h>

namespace uipc::backend
{
Buffer::Buffer(std::function<void(SizeT)> resize_func, std::function<BufferView()> get_buffer_view_func)
    : m_resize_func{resize_func}, m_get_buffer_view_func{get_buffer_view_func}
{
}

void Buffer::resize(SizeT size)
{
    m_resize_func(size);
}

BufferView Buffer::view() const
{
    return m_get_buffer_view_func();
}
}