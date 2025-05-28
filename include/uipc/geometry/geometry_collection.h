#pragma once
#include <concepts>
#include <uipc/geometry/geometry.h>
#include <uipc/geometry/geometry_slot.h>
#include <uipc/common/set.h>

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::core
{
class SceneFactory;
}

namespace uipc::geometry
{
class UIPC_CORE_API IGeometryCollection
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

class UIPC_CORE_API GeometryCollection : public IGeometryCollection
{
    friend class core::internal::Scene;
    friend class core::SceneFactory;
    friend class GeometryCollectionCommit;

  public:
    GeometryCollection() = default;

    GeometryCollection(const GeometryCollection&);
    GeometryCollection& operator=(const GeometryCollection&) = delete;

    GeometryCollection(GeometryCollection&&) noexcept   = default;
    GeometryCollection& operator=(GeometryCollection&&) = default;

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    S<geometry::GeometrySlotT<GeometryT>> emplace(const GeometryT& geometry);

    S<geometry::GeometrySlot> emplace(const geometry::Geometry& geometry);

    template <std::derived_from<geometry::Geometry> GeometryT>
        requires(!std::is_abstract_v<GeometryT>)
    S<geometry::GeometrySlotT<GeometryT>> pending_emplace(const GeometryT& geometry);

    template <std::derived_from<geometry::Geometry> GeometryT>
    S<geometry::GeometrySlotT<GeometryT>> find(IndexT id) noexcept;
    S<geometry::GeometrySlot>             find(IndexT id) noexcept;

    template <std::derived_from<geometry::Geometry> GeometryT>
    S<const geometry::GeometrySlotT<GeometryT>> find(IndexT id) const noexcept;
    S<const geometry::GeometrySlot>             find(IndexT id) const noexcept;

    void destroy(IndexT id) noexcept;
    void pending_destroy(IndexT id) noexcept;


    void solve_pending() noexcept;

    span<S<geometry::GeometrySlot>> geometry_slots() const noexcept;
    span<S<geometry::GeometrySlot>> pending_create_slots() const noexcept;
    span<IndexT>                    pending_destroy_ids() const noexcept;

  protected:
    virtual void   do_reserve(SizeT size) noexcept override;
    virtual void   do_clear() noexcept override;
    virtual SizeT  get_size() const noexcept override;
    virtual IndexT get_next_id() const noexcept override;

  private:
    unordered_map<IndexT, S<geometry::GeometrySlot>> m_geometries;
    unordered_map<IndexT, S<geometry::GeometrySlot>> m_pending_create;
    set<IndexT>                                      m_pending_destroy;

    IndexT m_next_id = 0;

    mutable bool m_dirty = true;

    mutable vector<S<geometry::GeometrySlot>> m_geometry_slots;
    mutable vector<S<geometry::GeometrySlot>> m_pending_create_slots;
    mutable vector<IndexT>                    m_pending_destroy_ids;

    void flush() const;

    void build_from(span<S<geometry::GeometrySlot>> slots) noexcept;
    void update_from(const unordered_map<IndexT, S<GeometryCommit>>& commits) noexcept;
};
}  // namespace uipc::geometry

#include "details/geometry_collection.inl"