#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/geometry/geometry.h>

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

  public:
    GeometrySlot(IndexT id) noexcept;
    virtual ~GeometrySlot() = default;
    IndexT            id() const noexcept;
    Geometry&         geometry() noexcept;
    const Geometry&   geometry() const noexcept;
    GeometrySlotState state() const noexcept;

    GeometrySlot(const GeometrySlot&)            = delete;
    GeometrySlot(GeometrySlot&&)                 = delete;
    GeometrySlot& operator=(const GeometrySlot&) = delete;
    GeometrySlot& operator=(GeometrySlot&&)      = delete;

  protected:
    virtual Geometry&       get_geometry() noexcept       = 0;
    virtual const Geometry& get_geometry() const noexcept = 0;

  private:
    IndexT            m_id;
    GeometrySlotState m_state = GeometrySlotState::Normal;
    void              state(GeometrySlotState state) noexcept;
};

UIPC_CORE_EXPORT_TEMPLATE_CLASS std::shared_ptr<GeometrySlot>;

template <std::derived_from<Geometry> GeometryT>
    requires(!std::is_abstract_v<GeometryT>)
class GeometrySlotT;
}  // namespace uipc::geometry
