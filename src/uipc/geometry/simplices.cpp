#include <uipc/geometry/simplices.h>
#include <numeric>

namespace uipc::geometry
{
SizeT ISimplices::size() const
{
    return get_size();
}
void ISimplices::resize(SizeT N)
{
    do_resize(N);
}
void ISimplices::clear()
{
    do_clear();
}
S<ISimplices> ISimplices::clone() const
{
    return do_clone();
}
void ISimplices::reserve(SizeT N)
{
    do_reserve(N);
}
IndexT Vertices::dim() const
{
    return 0;
}
std::span<const IndexT> Vertices::view() const
{
    if(m_size != m_simplices.size())
    {
        m_simplices.resize(m_size);
        std::iota(m_simplices.begin(), m_simplices.end(), 0);
    }
    return m_simplices;
}
std::span<IndexT> Vertices::view()
{
    if(m_size != m_simplices.size())
    {
        m_simplices.resize(m_size);
        std::iota(m_simplices.begin(), m_simplices.end(), 0);
    }
    return m_simplices;
}
SizeT Vertices::get_size() const
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
S<ISimplices> Vertices::do_clone() const
{
    return std::make_shared<Vertices>(*this);
}
void Vertices::do_reserve(SizeT N)
{
    // Do nothing
}
}  // namespace uipc::geometry
