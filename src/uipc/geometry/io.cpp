#include <uipc/geometry/io.h>
#include <uipc/geometry/factory.h>
#include <igl/readMSH.h>
#include <format>
#include <uipc/common/enumerate.h>
#include <filesystem>

namespace uipc::geometry
{
template <typename T>
using RowMajorMatrix =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Eigen::VectorXi;

SimplicialComplex read_msh(std::string_view file_name)
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
}  // namespace uipc::geometry
