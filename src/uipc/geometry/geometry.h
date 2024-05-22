#pragma once
#include <string>

namespace uipc::geometry
{
class IGeometry
{
  public:
    IGeometry(const std::string& type);
    std::string_view type() const;

  private:
    std::string m_type;
};
}  // namespace uipc::geometry
