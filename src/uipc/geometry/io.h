#pragma once
#include <uipc/common/exception.h>
#include <uipc/geometry/simplicial_complex.h>


namespace uipc::geometry
{
/**
 * @brief A class for reading and writing simplicial complex.
 */
class SimplicialComplexIO
{
  public:

    /**
     * @brief A unified interface for reading a simplicial complex from a file, the file type is determined by the file extension.
     * 
     * \param file_name The file to read
     * \return SimplicialComplex
     */
    [[nodiscard]] SimplicialComplex read(std::string_view file_name);

    /**
     * @brief Read a tetmesh from a .msh file.
     * 
     * @param file_name The file to read
     * 
     * @return SimplicialComplex 
     */
    [[nodiscard]] SimplicialComplex read_msh(std::string_view file_name);

    /**
	 * @brief Read a trimesh, linemesh or particles from a .obj file.
	 * 
	 * @param file_name The file to read
	 * 
	 * @return  SimplicialComplex
	 */
	[[nodiscard]] SimplicialComplex read_obj(std::string_view file_name);
};
}  // namespace uipc::geometries

namespace uipc::geometry
{
class GeometryIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometries