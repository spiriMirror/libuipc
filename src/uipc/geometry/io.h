#pragma once
#include <uipc/common/exception.h>
#include <uipc/geometry/simplicial_complex.h>


namespace uipc::geometry
{
/**
 * @brief Read a tetmesh from a .msh file.
 * 
 * @param file_name The file to read
 * @return SimplicialComplex 
 */
[[nodiscard]] SimplicialComplex read_msh(std::string_view file_name);

/**
 * @brief Read a trimesh from a .obj file.
 * 
 * @param file_name The file to read
 */
[[nodiscard]] SimplicialComplex read_obj(std::string_view file_name);
}

namespace uipc::geometry
{
class GeometryIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry