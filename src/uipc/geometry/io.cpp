#include <uipc/geometry/io.h>
#include <uipc/geometry/factory.h>
#include <igl/readMSH.h>
#include <format>
#include <uipc/common/enumerate.h>
#include <filesystem>
#include <igl/read_triangle_mesh.h>

namespace uipc::geometry
{
template <typename T>
using RowMajorMatrix =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Eigen::VectorXi;

namespace fs = std::filesystem;

SimplicialComplex SimplicialComplexIO::read(std::string_view file_name)
{
    fs::path path{file_name};
    if(path.extension() == ".msh")
    {
        return read_msh(file_name);
    }
    else if(path.extension() == ".obj")
    {
        return read_obj(file_name);
    }
    else
    {
        throw GeometryIOError{std::format("Unsupported file format: {}", file_name)};
    }
}

SimplicialComplex SimplicialComplexIO::read_msh(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{std::format("File does not exist: {}", file_name)};
    }
    RowMajorMatrix<Float>  X;
    RowMajorMatrix<IndexT> F;
    RowMajorMatrix<IndexT> T;
    VectorXi               TriTag;
    VectorXi               TetTag;
    if(!igl::readMSH(std::string{file_name}, X, F, T, TriTag, TetTag))
    {
        throw GeometryIOError{std::format("Failed to load .msh file: {}", file_name)};
    }
    vector<Vector3> Vs;
    Vs.resize(X.rows());
    for(auto&& [i, v] : enumerate(Vs))
        v = X.row(i);
    vector<Vector4i> Ts;
    Ts.resize(T.rows());
    for(auto&& [i, t] : enumerate(Ts))
        t = T.row(i);
    return tetmesh(Vs, Ts);
}

SimplicialComplex SimplicialComplexIO::read_obj(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{std::format("File does not exist: {}", file_name)};
    }
    // TODO: We may want to take more information from the .obj file
    RowMajorMatrix<Float>  X;
    RowMajorMatrix<IndexT> F;
    if(!igl::read_triangle_mesh(std::string{file_name}, X, F))
    {
        throw GeometryIOError{std::format("Failed to load .obj file: {}", file_name)};
    }
    vector<Vector3> Vs;
    Vs.resize(X.rows());
    for(auto&& [i, v] : enumerate(Vs))
        v = X.row(i);
    vector<Vector3i> Fs;
    Fs.resize(F.rows());
    for(auto&& [i, f] : enumerate(Fs))
        f = F.row(i);
    return trimesh(Vs, Fs);
}
}  // namespace uipc::geometry
