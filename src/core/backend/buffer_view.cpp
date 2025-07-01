#include <uipc/backend/buffer_view.h>
#include <uipc/common/log.h>
#include <uipc/common/const_string_map.h>

namespace uipc::backend
{
static thread_local ConstStringMap m_backend_name_map;

BufferView::BufferView(HandleT          handle,
                       SizeT            offset,
                       SizeT            size,
                       SizeT            element_size,
                       SizeT            element_stride,
                       std::string_view backend_name) noexcept
    : m_handle(handle)
    , m_offset(offset)
    , m_size(size)
    , m_element_size(element_size)
    , m_element_stride(element_stride)
    , m_backend(m_backend_name_map[backend_name])
{
    UIPC_ASSERT(element_stride >= element_size,
                "[{}]: Element stride({}) must be greater or equal to element size({}). ",
                backend_name,
                element_stride,
                element_size);
}

HandleT BufferView::handle() const noexcept
{
    return m_handle;
}

SizeT BufferView::offset() const noexcept
{
    return m_offset;
}

SizeT BufferView::size() const noexcept
{
    return m_size;
}

SizeT BufferView::element_size() const noexcept
{
    return m_element_size;
}

SizeT BufferView::element_stride() const noexcept
{
    return m_element_stride;
}

SizeT BufferView::size_in_bytes() const noexcept
{
    return m_size * m_element_size;
}

std::string_view BufferView::backend() const noexcept
{
    return m_backend;
}

BufferView::operator bool() const noexcept
{
    return m_offset != INVALID;
}

BufferView BufferView::subview(SizeT offset, SizeT size) const noexcept
{
    UIPC_ASSERT(offset + size <= m_size,
                "[{}]: Subview({},{}) out of bounds({}).",
                m_backend,
                offset,
                size,
                m_size);

    return BufferView{m_handle, m_offset + offset * m_element_stride, size, m_element_size, m_element_stride, m_backend};
}
}  // namespace uipc::backend
