#pragma once
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/common/set.h>

namespace uipc::geometry
{
class IGeometryCollection
{
  public:
    virtual ~IGeometryCollection() = default;
    [[nodiscard]] SizeT size() const noexcept;
    void                clear() noexcept;
    void                reserve(SizeT size) noexcept;
    IndexT              next_id() const noexcept;

  protected:
    virtual SizeT  get_size() const noexcept       = 0;
    virtual void   do_clear() noexcept             = 0;
    virtual void   do_reserve(SizeT size) noexcept = 0;
    virtual IndexT get_next_id() const noexcept    = 0;
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
    P<geometry::GeometrySlot<GeometryT>> pending_emplace(const GeometryT& geometry);

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    P<geometry::GeometrySlot<GeometryT>> find(IndexT id) noexcept;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    P<const geometry::GeometrySlot<GeometryT>> find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;
    void pending_destroy(IndexT id) noexcept;

    void solve_pending() noexcept;

  protected:
    virtual void   do_reserve(SizeT size) noexcept override;
    virtual void   do_clear() noexcept override;
    virtual SizeT  get_size() const noexcept override;
    virtual IndexT get_next_id() const noexcept override;

  private:
    unordered_map<IndexT, S<geometry::IGeometrySlot>> m_geometries;
    unordered_map<IndexT, S<geometry::IGeometrySlot>> m_pending_create;
    set<IndexT>                                       m_pending_destroy;

    IndexT m_next_id = 0;
};
}  // namespace uipc::geometry

#include "details/geometry_collection.inl"