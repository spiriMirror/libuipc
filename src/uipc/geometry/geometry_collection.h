#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>

namespace uipc::geometry
{
class IGeometryCollection
{
  public:
    virtual ~IGeometryCollection() = default;
    [[nodiscard]] SizeT size() const noexcept;
    void                clear() noexcept;
    void                reserve(SizeT size) noexcept;

  protected:
    virtual SizeT get_size() const noexcept       = 0;
    virtual void  do_clear() noexcept             = 0;
    virtual void  do_reserve(SizeT size) noexcept = 0;
};

class GeometryCollection : public IGeometryCollection
{
  public:
    GeometryCollection() = default;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    P<geometry::GeometrySlot<GeometryT>> emplace(const GeometryT& geometry);

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    P<geometry::GeometrySlot<GeometryT>> find(IndexT id) noexcept;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    P<const geometry::GeometrySlot<GeometryT>> find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;

  protected:
    virtual void  do_reserve(SizeT size) noexcept override;
    virtual void  do_clear() noexcept override;
    virtual SizeT get_size() const noexcept override;

  private:
    unordered_map<IndexT, S<geometry::IGeometrySlot>> m_geometries;
};
}  // namespace uipc::world

#include "details/geometry_collection.inl"