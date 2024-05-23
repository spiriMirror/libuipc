#pragma once
#include <string_view>

namespace uipc::geometry
{
/**
 * @brief An abstract class for geometry
 */
class IGeometry
{
  public:
  /**
   * @brief Get the type of the geometry, check the type to downcast the geometry to a specific type
   * 
   * @return a string_view of the type of the geometry
   */
    [[nodiscard]] std::string_view type() const;
    virtual ~IGeometry()                  = default;
  protected:
    [[nodiscard]] virtual std::string_view get_type() const = 0;
};
}  // namespace uipc::geometry
