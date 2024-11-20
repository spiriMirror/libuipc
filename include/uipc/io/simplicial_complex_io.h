#pragma once
#include <uipc/common/exception.h>
#include <uipc/geometry/simplicial_complex.h>


namespace uipc::geometry
{
/**
 * @brief A class for reading and writing simplicial complex.
 */
class UIPC_IO_API SimplicialComplexIO
{
  public:
    SimplicialComplexIO() = default;
    explicit SimplicialComplexIO(const Matrix4x4& pre_transform) noexcept;
    explicit SimplicialComplexIO(const Transform& pre_transform) noexcept;

    // default copy/move ctor/assignment
    SimplicialComplexIO(const SimplicialComplexIO&)            = default;
    SimplicialComplexIO(SimplicialComplexIO&&)                 = default;
    SimplicialComplexIO& operator=(const SimplicialComplexIO&) = default;
    SimplicialComplexIO& operator=(SimplicialComplexIO&&)      = default;

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

    /**
     * @brief Read a trimesh, linemesh or particles from a .ply file.
     * 
     * @param file_name The file to read
     * 
     * @return  SimplicialComplex
     */
    [[nodiscard]] SimplicialComplex read_ply(std::string_view file_name);


    /**
     * @brief Write a simplicial complex to a file, the file type is determined by the file extension.
     * 
     * @param file_name The file to write
     * @param sc The simplicial complex to write
     */
    void write(std::string_view file_name, const SimplicialComplex& sc);

    /**
     * @brief Write a simplicial complex to .obj file.
     * 
     * @param file_name The file to write
     * @param sc The simplicial complex to write
     */
    void write_obj(std::string_view file_name, const SimplicialComplex& sc);

    /**
     * @brief Write a simplicial complex to .msh file.
     * 
     * @param file_name The file to write
     * @param sc The simplicial complex to write
     */
    void write_msh(std::string_view file_name, const SimplicialComplex& sc);

  private:
    Matrix4x4 m_pre_transform = Matrix4x4::Identity();
    void      apply_pre_transform(Vector3& v) const noexcept;
};
}  // namespace uipc::geometry

namespace uipc::geometry
{
class GeometryIOError : public Exception
{
  public:
    using Exception::Exception;
};
}  // namespace uipc::geometry