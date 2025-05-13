#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/geometry.h>

namespace uipc::core
{
class SceneFactory;
}

namespace uipc::geometry
{
enum class GeometrySlotState
{
    Normal,
    PendingCreate,
    PendingDestroy
};

class UIPC_CORE_API GeometrySlot
{
    friend class GeometryCollection;
    friend class core::SceneFactory;
    friend class GeometryAtlas;

  public:
    GeometrySlot(IndexT id) noexcept;
    virtual ~GeometrySlot() = default;
    IndexT          id() const noexcept;
    Geometry&       geometry() noexcept;
    const Geometry& geometry() const noexcept;

    GeometrySlotState state() const noexcept;
    S<GeometrySlot>   clone() const;

    GeometrySlot(const GeometrySlot&)            = delete;
    GeometrySlot(GeometrySlot&&)                 = delete;
    GeometrySlot& operator=(const GeometrySlot&) = delete;
    GeometrySlot& operator=(GeometrySlot&&)      = delete;

  protected:
    virtual Geometry&       get_geometry() noexcept       = 0;
    virtual const Geometry& get_geometry() const noexcept = 0;
    virtual S<GeometrySlot> do_clone() const              = 0;

  private:
    IndexT            m_id;
    void              id(IndexT id) noexcept;
    GeometrySlotState m_state = GeometrySlotState::Normal;
    void              state(GeometrySlotState state) noexcept;
};

template <std::derived_from<Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
class GeometrySlotT;

template <>
class UIPC_CORE_API GeometrySlotT<Geometry> : public GeometrySlot
{
  public:
    GeometrySlotT(IndexT id, const Geometry& geometry);

  protected:
    Geometry&       get_geometry() noexcept override;
    const Geometry& get_geometry() const noexcept override;
    S<GeometrySlot> do_clone() const override;

  private:
    Geometry m_geometry;
};
}  // namespace uipc::geometry

namespace std
{
// clang++-17: explicit instantiation of 'shared_ptr' should be in a namespace enclosing 'std'.
extern template class std::shared_ptr<uipc::geometry::GeometrySlot>;
}  // namespace std