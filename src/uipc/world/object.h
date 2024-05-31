#pragma once
#include <string_view>
#include <uipc/common/vector.h>
#include <uipc/geometry/geometry.h>
#include <uipc/common/unordered_map.h>
#include <uipc/geometry/geometry_collection.h>
#include <uipc/common/type_traits.h>

namespace uipc::world
{
class IObject
{
  public:
    virtual ~IObject() = default;
    [[nodiscard]] std::string_view name() const noexcept;

  protected:
    virtual std::string_view get_name() const noexcept = 0;
};

class Object;

template <std::derived_from<geometry::Geometry> GeometryT>
class ObjectGeometrySlots
{
    using NonConstGeometryT = std::remove_const_t<GeometryT>;
    using AutoGeometrySlot =
        typename propagate_const<GeometryT, geometry::GeometrySlot<NonConstGeometryT>>::type;

  public:
    P<AutoGeometrySlot> geometry;
    P<AutoGeometrySlot> rest_geometry;
};

class Object : public IObject
{
  public:
    class Geometries
    {
        friend class Object;

      public:
        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<GeometryT> find(IndexT id) &&;


        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<GeometryT> create(const GeometryT& geometry) &&;

        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<GeometryT> create(const GeometryT& geometry,
                                                            const GeometryT& rest_geometry) &&;

      private:
        Geometries(Object& object) noexcept;
        Object& m_object;
    };

    class CGeometries
    {
        friend class Object;

      public:
        template <std::derived_from<geometry::Geometry> GeometryT>
            requires(!std::is_abstract_v<GeometryT>)
        [[nodiscard]] ObjectGeometrySlots<const GeometryT> find(IndexT id) &&;

      private:
        CGeometries(const Object& object) noexcept;
        const Object& m_object;
    };

    Object(std::string_view name = "") noexcept;
    Object(Object&&) = default;

    Object(const Object&)            = delete;
    Object& operator=(const Object&) = delete;
    Object& operator=(Object&&)      = delete;

    Geometries  geometries();
    CGeometries geometries() const;

  protected:
    [[nodiscard]] std::string_view get_name() const noexcept override;

  private:
    std::string                  m_name;
    geometry::GeometryCollection m_geometries;
    geometry::GeometryCollection m_rest_geometries;
};
}  // namespace uipc::world


#include "details/object.inl"