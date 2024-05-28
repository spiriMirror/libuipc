#include <uipc/geometry/simplices.h>
#include <numeric>

namespace uipc::geometry
{
IndexT ISimplices::dim() const
{
    return get_dim();
}

backend::BufferView Vertices::get_backend_view() const noexcept
{
    return m_backend_view;
}

IndexT Vertices::get_dim() const
{
    return 0;
}

S<ISimplices> ISimplices::clone() const
{
    return std::static_pointer_cast<ISimplices>(do_clone());
}

SizeT ISimplices::get_tuple_size() const noexcept
{
    return dim() + 1ull;
}

SizeT ISimplices::get_tuple_size(IndexT i) const noexcept
{
    return get_tuple_size();
}

span<const IndexT> Vertices::view() const
{
    if(m_size != m_simplices.size())
    {
        m_simplices.resize(m_size);
        std::iota(m_simplices.begin(), m_simplices.end(), 0);
    }
    return m_simplices;
}

span<IndexT> Vertices::view()
{
    if(m_size != m_simplices.size())
    {
        m_simplices.resize(m_size);
        std::iota(m_simplices.begin(), m_simplices.end(), 0);
    }
    return m_simplices;
}

SizeT Vertices::get_size() const noexcept
{
    return m_size;
}

void Vertices::do_resize(SizeT N)
{
    m_size = N;
}

void Vertices::do_clear()
{
    m_size = 0;
}

S<ITopoElements> Vertices::do_clone() const
{
    return std::make_shared<Vertices>(*this);
}

void Vertices::do_reserve(SizeT N)
{
    // Do nothing
}

span<IndexT> view(Vertices& vertices) noexcept
{
    return vertices.view();
}
}  // namespace uipc::geometry
