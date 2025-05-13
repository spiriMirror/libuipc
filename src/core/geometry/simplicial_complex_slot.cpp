#include <uipc/geometry/simplicial_complex_slot.h>

namespace uipc::geometry
{
GeometrySlotT<SimplicialComplex>::GeometrySlotT(IndexT id, const SimplicialComplex& simplicial_complex) noexcept
    : GeometrySlot{id}
    , m_simplicial_complex{simplicial_complex}
{
}

SimplicialComplex& GeometrySlotT<SimplicialComplex>::geometry() noexcept
{
    return m_simplicial_complex;
}

const SimplicialComplex& GeometrySlotT<SimplicialComplex>::geometry() const noexcept
{
    return m_simplicial_complex;
}

Geometry& GeometrySlotT<SimplicialComplex>::get_geometry() noexcept
{
    return m_simplicial_complex;
}

const Geometry& GeometrySlotT<SimplicialComplex>::get_geometry() const noexcept
{
    return m_simplicial_complex;
}

S<GeometrySlot> GeometrySlotT<SimplicialComplex>::do_clone() const
{
    return uipc::make_shared<GeometrySlotT<SimplicialComplex>>(id(), m_simplicial_complex);
}

template class UIPC_CORE_API GeometrySlotT<SimplicialComplex>;
}  // namespace uipc::geometry

namespace std
{
template class UIPC_CORE_API std::shared_ptr<uipc::geometry::SimplicialComplexSlot>;
}
