#pragma once
#include <uipc/common/exception.h>
#include <uipc/geometry/simplicial_complex.h>


namespace uipc::geometry
{
[[nodiscard]] SimplicialComplex read_msh(std::string_view file_name);
}

namespace uipc::geometry
{
class GeometryIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry