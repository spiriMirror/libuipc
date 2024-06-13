#pragma once
#include <string_view>
#include <uipc/common/vector.h>
#include <uipc/geometry/geometry.h>
#include <uipc/common/unordered_map.h>
#include <uipc/common/type_traits.h>
#include <uipc/geometry/geometry_collection.h>

namespace uipc::world
{
class Scene;

class UIPC_CORE_API IObject
{
  public:
    virtual ~IObject() = default;
    [[nodiscard]] std::string_view name() const noexcept;
    [[nodiscard]] IndexT           id() const noexcept;

  protected:
    virtual std::string_view get_name() const noexcept = 0;
    virtual IndexT           get_id() const noexcept   = 0;
};

class Object;

template <std::derived_from<geometry::Geometry> GeometryT>
class ObjectGeometrySlots
{
    using NonConstGeometryT = std::remove_const_t<GeometryT>;
    using AutoGeometrySlot =
        typename propagate_const<GeometryT, geometry::GeometrySlotT<NonConstGeometryT>>::type;

  public:
    P<AutoGeometrySlot> geometry;
    P<AutoGeometrySlot> rest_geometry;
};

template <>
class ObjectGeometrySlots<geometry::Geometry>
{
  public:
    P<geometry::GeometrySlot> geometry;
    P<geometry::GeometrySlot> rest_geometry;
};

UIPC_CORE_EXPORT_TEMPLATE_CLASS ObjectGeometrySlots<geometry::Geometry>;

template <>
class ObjectGeometrySlots<const geometry::Geometry>
{
  public:
    P<const geometry::GeometrySlot> geometry;
    P<const geometry::GeometrySlot> rest_geometry;
};

UIPC_CORE_EXPORT_TEMPLATE_CLASS ObjectGeometrySlots<const geometry::Geometry>;

class UIPC_CORE_API Object : public IObject
{
    friend class Scene;

  public:
    class UIPC_CORE_API Geometries
    {
        friend class Object;

      public:
        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<GeometryT> create(const GeometryT& geometry) &&;

        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<GeometryT> create(const GeometryT& geometry,
                                                            const GeometryT& rest_geometry) &&;

        span<const IndexT> ids() && noexcept;

      private:
        Geometries(Object& object) noexcept;
        Object& m_object;
    };

    class UIPC_CORE_API CGeometries
    {
        friend class Object;

      public:
        span<const IndexT> ids() && noexcept;

      private:
        CGeometries(const Object& object) noexcept;
        const Object& m_object;
    };

    Object(Scene& scene, IndexT id, std::string_view name = "") noexcept;
    Object(Object&&) = default;

    Object(const Object&)            = delete;
    Object& operator=(const Object&) = delete;
    Object& operator=(Object&&)      = delete;

    [[nodiscard]] Geometries  geometries() noexcept;
    [[nodiscard]] CGeometries geometries() const noexcept;

  protected:
    [[nodiscard]] std::string_view get_name() const noexcept override;
    [[nodiscard]] IndexT           get_id() const noexcept override;

  private:
    Scene&                        m_scene;
    IndexT                        m_id;
    std::string                   m_name;
    vector<IndexT>                m_geometry_ids;
    geometry::GeometryCollection& geometry_collection() noexcept;
    geometry::GeometryCollection& rest_geometry_collection() noexcept;
    bool                          scene_started() const noexcept;
};
}  // namespace uipc::world


#include "details/object.inl"