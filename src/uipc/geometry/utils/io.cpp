#include <uipc/geometry/io.h>
#include <uipc/geometry/factory.h>
#include <igl/readMSH.h>
#include <uipc/common/format.h>
#include <uipc/common/enumerate.h>
#include <filesystem>
#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>

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
        throw GeometryIOError{fmt::format("Unsupported file format: {}", file_name)};
    }
}

SimplicialComplex SimplicialComplexIO::read_msh(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{fmt::format("File does not exist: {}", file_name)};
    }
    RowMajorMatrix<Float>  X;
    RowMajorMatrix<IndexT> F;
    RowMajorMatrix<IndexT> T;
    VectorXi               TriTag;
    VectorXi               TetTag;
    if(!igl::readMSH(std::string{file_name}, X, F, T, TriTag, TetTag))
    {
        throw GeometryIOError{fmt::format("Failed to load .msh file: {}", file_name)};
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
        throw GeometryIOError{fmt::format("File does not exist: {}", file_name)};
    }
    // TODO: We may want to take more information from the .obj file
    RowMajorMatrix<Float>  X;
    RowMajorMatrix<IndexT> F;
    if(!igl::read_triangle_mesh(std::string{file_name}, X, F))
    {
        throw GeometryIOError{fmt::format("Failed to load .obj file: {}", file_name)};
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

void SimplicialComplexIO::write_obj(std::string_view file_name, const SimplicialComplex& sc)
{
    if(sc.dim() > 2)
    {
        throw GeometryIOError{fmt::format("Cannot write simplicial complex of dimension {} to .obj file",
                                          sc.dim())};
    }

    auto E_size = sc.edges().size();

    auto Ps = sc.positions().view();
    auto Fs = sc.triangles().topo().view();


    RowMajorMatrix<Float>  X{Ps.size(), 3};
    RowMajorMatrix<IndexT> F{Fs.size(), 3};

    for(auto&& [i, p] : enumerate(Ps))
        X.row(i) = p;
    for(auto&& [i, f] : enumerate(Fs))
        F.row(i) = f;

    if(!igl::writeOBJ(std::string{file_name}, X, F))
    {
        throw GeometryIOError{fmt::format("Failed to write .obj file: {}", file_name)};
    }
}
}  // namespace uipc::geometry
