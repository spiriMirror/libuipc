#pragma once
#include <string_view>
#include <uipc/geometry/attribute_collection.h>

namespace uipc::geometry
{
/**
 * @brief An abstract class for geometries
 */
class IGeometry
{
  public:
    /**
     * @brief Get the type of the geometries, check the type to downcast the geometries to a specific type
     * 
     * @return a string_view of the type of the geometries
     */
    [[nodiscard]] std::string_view type() const noexcept;
    virtual ~IGeometry() = default;

  protected:
    [[nodiscard]] virtual std::string_view get_type() const noexcept = 0;
};

/**
 * @brief A base geometries class that contains the instance attributes and the meta attributes.
 */
class Geometry : public IGeometry
{
  public:
    /**
     * @brief A wrapper class for the meta attributes of a geometries.
     */
    class MetaAttributes
    {
      public:
        MetaAttributes(AttributeCollection& attributes);
        MetaAttributes(const MetaAttributes& o)            = default;
        MetaAttributes(MetaAttributes&& o)                 = default;
        MetaAttributes& operator=(const MetaAttributes& o) = default;
        MetaAttributes& operator=(MetaAttributes&& o)      = default;

        /**
         * @brief Find an attribute by type and name, if the attribute does not exist, return nullptr.
         */
        template <typename T>
        [[nodiscard]] auto find(std::string_view name)
        {
            return m_attributes.template find<T>(name);
        }

        /**
         * @brief Create an attribute with the given name.
         */
        template <typename T>
        decltype(auto) create(std::string_view name, const T& init_value = {})
        {
            return m_attributes.template create<T>(name, init_value);
        }

      private:
        AttributeCollection& m_attributes;
    };

    /**
     * @brief A wrapper class for the instance attributes of a geometries.
     */
    class InstanceAttributes
    {
      public:
        InstanceAttributes(AttributeCollection& attributes);
        InstanceAttributes(const InstanceAttributes& o)            = default;
        InstanceAttributes(InstanceAttributes&& o)                 = default;
        InstanceAttributes& operator=(const InstanceAttributes& o) = default;
        InstanceAttributes& operator=(InstanceAttributes&& o)      = default;

        /**
         * @sa [AttributeCollection::resize()](../../AttributeCollection/#resize)
         */
        void resize(size_t size);
        /**
         * @sa [AttributeCollection::reserve()](../../AttributeCollection/#reserve)
         */
        void reserve(size_t size);
        /**
         * @sa [AttributeCollection::clear()](../../AttributeCollection/#clear)
         */
        void clear();
        /**
         * @sa [AttributeCollection::size()](../../AttributeCollection/#size)
         */
        [[nodiscard]] SizeT size() const;
        /**
         * @sa [AttributeCollection::destroy()](../../AttributeCollection/#destroy) 
         */
        void destroy(std::string_view name);

        /**
         * @brief Find an attribute by type and name, if the attribute does not exist, return empty OptionalRef.
         */
        template <typename T>
        [[nodiscard]] auto find(std::string_view name)
        {
            return m_attributes.template find<T>(name);
        }

        /**
         * @brief Find an attribute by type and name, if the attribute does not exist, return empty OptionalRef.
         */
        template <typename T>
        [[nodiscard]] auto find(std::string_view name) const
        {
            return std::as_const(m_attributes).template find<T>(name);
        }

        /**
         * @brief Create an attribute with the given name.
         */
        template <typename T>
        decltype(auto) create(std::string_view name, const T& init_value = {})
        {
            return m_attributes.template create<T>(name, init_value);
        }

      private:
        AttributeCollection& m_attributes;
    };

    Geometry();

    // allow copy and move on construction, because the geometry truely empty
    Geometry(const Geometry& o) = default;
    Geometry(Geometry&& o)      = default;

    // no copy or move assignment, because the geometry is no longer empty
    Geometry& operator=(const Geometry& o) = delete;
    Geometry& operator=(Geometry&& o)      = delete;

    /**
     * @brief A short-cut to get the non-const transforms attribute slot.
     * 
     * @return The attribute slot of the non-const transforms.
     */
    [[nodiscard]] AttributeSlot<Matrix4x4>& transforms();
    /**
     * @brief A short-cut to get the const transforms attribute slot.
     * 
     * @return The attribute slot of the const transforms.
     */
    [[nodiscard]] const AttributeSlot<Matrix4x4>& transforms() const;

    /**
     * @brief Get the meta attributes of the geometries.
     * 
     * @return The meta attributes of the geometries. 
     */
    [[nodiscard]] MetaAttributes meta();
    /**
     * @brief Get the instance attributes of the geometries.
     * 
     * @return  The instance attributes of the geometries.
     */
    [[nodiscard]] InstanceAttributes instances();

  private:
    AttributeCollection m_intances;
    AttributeCollection m_meta;
};
}  // namespace uipc::geometry
