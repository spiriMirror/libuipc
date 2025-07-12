#include <uipc/io/simplicial_complex_io.h>
#include <uipc/geometry/utils/factory.h>
#include <uipc/common/list.h>
#include <igl/readMSH.h>
#include <uipc/common/format.h>
#include <uipc/common/enumerate.h>
#include <filesystem>
#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>
#include <uipc/builtin/attribute_name.h>
#include <Eigen/Geometry>
#include <igl/readSTL.h>
#include <igl/readPLY.h>
#include <igl/readOBJ.h>
#include "stl_reader.h"
#include <igl/remove_duplicate_vertices.h>


namespace uipc::geometry
{
SimplicialComplexIO::SimplicialComplexIO(const Matrix4x4& pre_transform) noexcept
    : m_pre_transform{pre_transform}
{
}

SimplicialComplexIO::SimplicialComplexIO(const Transform& pre_transform) noexcept
    : m_pre_transform{pre_transform.matrix()}
{
}

template <typename T>
using RowMajorMatrix =
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Eigen::VectorXi;

namespace fs = std::filesystem;


void SimplicialComplexIO::apply_pre_transform(Vector3& v) const noexcept
{
    v = (m_pre_transform * v.homogeneous()).head<3>();
}

SimplicialComplex SimplicialComplexIO::read(std::string_view file_name)
{
    fs::path path{file_name};
    auto     ext = path.extension().string();
    // lowercase the extension
    std::ranges::transform(ext, ext.begin(), ::tolower);
    if(ext == ".msh")
    {
        return read_msh(file_name);
    }
    else if(ext == ".obj")
    {
        return read_obj(file_name);
    }
    else if(ext == ".ply")
    {
        return read_ply(file_name);
    }
    else if(ext == ".stl")
    {
        return read_stl(file_name);
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

    RowMajorMatrix<double> doubleX;
    RowMajorMatrix<Float>  X;

    RowMajorMatrix<IndexT> F;
    RowMajorMatrix<IndexT> T;
    VectorXi               TriTag;
    VectorXi               TetTag;
    if(!igl::readMSH(string{file_name}, doubleX, F, T, TriTag, TetTag))
    {
        throw GeometryIOError{fmt::format("Failed to load .msh file: {}", file_name)};
    }

    if constexpr(std::is_same_v<double, Float>)
    {
        X = std::move(doubleX);
    }
    else
    {
        X = doubleX.cast<Float>();
    }
    vector<Vector3> Vs;
    Vs.resize(X.rows());
    for(auto&& [i, v] : enumerate(Vs))
    {
        v = X.row(i);
        apply_pre_transform(v);
    }
    vector<Vector4i> Ts;
    Ts.resize(T.rows());
    for(auto&& [i, t] : enumerate(Ts))
        t = T.row(i);
    return tetmesh(Vs, Ts);
}

static SimplicialComplex obj_trimesh(std::string_view    file_name,
                                     IndexT              deg3_count,
                                     IndexT              deg4_count,
                                     span<const Vector3> Xs,
                                     const VectorXi&     Is,
                                     const VectorXi&     Cs)
{
    IndexT tri_count = deg3_count + deg4_count * 2;  // each deg4 polygon is split into 2 triangles
    vector<Vector3i> Fs;
    Fs.reserve(tri_count);
    UIPC_ASSERT(Cs[0] == 0,
                "Cs[0] must be 0, got: {} in file {}, why can it happen, something wrong with libigl::readOBJ?",
                Cs[0],
                file_name);

    IndexT offset = Cs[0];
    for(IndexT i = 0; i < Cs.size() - 1; ++i)
    {
        IndexT deg = Cs[i + 1] - Cs[i];
        if(deg == 3)  // triangle
        {
            Fs.push_back(Vector3i{Is[offset + 0], Is[offset + 1], Is[offset + 2]});
        }
        else if(deg == 4)  // quad, split into two triangles
        {
            Fs.push_back(Vector3i{Is[offset + 0], Is[offset + 1], Is[offset + 2]});
            Fs.push_back(Vector3i{Is[offset + 0], Is[offset + 2], Is[offset + 3]});
        }
        else  // no error but skip for robust
        {
            vector<IndexT> indices;
            indices.resize(deg);
            for(IndexT j = 0; j < deg; ++j)
            {
                indices[j] = Is[offset + j];
            }
            spdlog::warn("Unsupported polygon degree: {}  (face [{}]) in file: {}, skipping it.",
                         deg,
                         fmt::join(indices, ", "),
                         file_name);
        }
        offset += deg;  // move to the next polygon
    }

    return trimesh(Xs, Fs);
}

static SimplicialComplex obj_linemesh(std::string_view    file_name,
                                      IndexT              deg2_count,
                                      span<const Vector3> Xs,
                                      const VectorXi&     Is,
                                      const VectorXi&     Cs)
{
    IndexT           edge_count = deg2_count;
    vector<Vector2i> Es;
    Es.reserve(edge_count);
    UIPC_ASSERT(Cs[0] == 0,
                "Cs[0] must be 0, got: {} in file {}, why can it happen, something wrong with libigl::readOBJ?",
                Cs[0],
                file_name);
    IndexT offset = Cs[0];
    for(IndexT i = 0; i < Cs.size() - 1; ++i)
    {
        IndexT deg = Cs[i + 1] - Cs[i];
        if(deg == 2)  // line segment
        {
            Es.push_back(Vector2i{Is[offset + 0], Is[offset + 1]});
        }
        else  // no error but skip for robust
        {
            vector<IndexT> indices;
            indices.resize(deg);
            for(IndexT j = 0; j < deg; ++j)
            {
                indices[j] = Is[offset + j];
            }
            spdlog::warn("Unsupported polygon degree: {}  (edge [{}]) in file: {}, skipping it.",
                         deg,
                         fmt::join(indices, ", "),
                         file_name);
        }
        offset += deg;  // move to the next polygon
    }

    return linemesh(Xs, Es);
}

SimplicialComplex SimplicialComplexIO::read_obj(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{fmt::format("File does not exist: {}", file_name)};
    }

    RowMajorMatrix<Float> Xs;
    VectorXi              Is;
    VectorXi              Cs;

    if(!igl::readOBJ(string{file_name}, Xs, Is, Cs))
    {
        throw GeometryIOError{fmt::format("Failed to load .obj file: {}", file_name)};
    }

    vector<Vector3> Vs;
    Vs.resize(Xs.rows());
    for(auto&& [i, v] : enumerate(Vs))
    {
        v = Xs.row(i);
        apply_pre_transform(v);
    }

    if(Is.size() == 0)  // point cloud
    {
        return pointcloud(Vs);
    }

    UIPC_ASSERT(Cs.size() >= 2, "must have at least 1 component");

    vector<IndexT> poly_deg2count;
    poly_deg2count.resize(5, 0);

    for(IndexT i = 0; i < Cs.size() - 1; ++i)
    {
        IndexT deg = Cs[i + 1] - Cs[i];
        UIPC_ASSERT(deg >= 2, "poly deg({}) < 2, why can it happen?", deg);
        if(deg >= poly_deg2count.size())
        {
            poly_deg2count.resize(deg + 1);
            poly_deg2count[deg] = 0;  // initialize new degree count
        }
        poly_deg2count[deg] += 1;
    }

    if(poly_deg2count[3] || poly_deg2count[4])  // trimesh
    {
        return obj_trimesh(file_name, poly_deg2count[3], poly_deg2count[4], Vs, Is, Cs);
    }

    if(poly_deg2count[2] == 0)  // point cloud
    {
        return obj_linemesh(file_name, poly_deg2count[2], Vs, Is, Cs);
    }

    throw GeometryIOError{fmt::format(R"(Unsupported .obj file: {}. 
Only point cloud, line mesh and triangle mesh are supported. But in this file:
Polygon degree statistics -> [{}])",
                                      file_name,
                                      fmt::join(poly_deg2count, ", "))};  // only point cloud, line mesh and triangle mesh are supported
}

SimplicialComplex SimplicialComplexIO::read_ply(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{fmt::format("File does not exist: {}", file_name)};
    }

    RowMajorMatrix<Float>  X;
    RowMajorMatrix<IndexT> F;
    if(!igl::readPLY(string{file_name}, X, F))
    {
        throw GeometryIOError{fmt::format("Failed to load .ply file: {}", file_name)};
    }

    vector<Vector3> Vs;
    Vs.resize(X.rows());
    for(auto&& [i, v] : enumerate(Vs))
    {
        v = X.row(i);
        apply_pre_transform(v);
    }

    vector<Vector3i> Fs;
    Fs.resize(F.rows());
    for(auto&& [i, f] : enumerate(Fs))
        f = F.row(i);

    return trimesh(Vs, Fs);
}

SimplicialComplex SimplicialComplexIO::read_stl(std::string_view file_name)
{
    if(!std::filesystem::exists(file_name))
    {
        throw GeometryIOError{fmt::format("File does not exist: {}", file_name)};
    }

    stl_reader::StlMesh<Float, IndexT> mesh{std::string{file_name}};
    span<const Vector3>  Vs{reinterpret_cast<const Vector3*>(mesh.raw_coords()),
                           mesh.num_vrts()};
    span<const Vector3i> Fs{reinterpret_cast<const Vector3i*>(mesh.raw_tris()),
                            mesh.num_tris()};

    RowMajorMatrix<Float>  V(Vs.size(), 3);
    RowMajorMatrix<IndexT> F(Fs.size(), 3);


    for(auto&& [i, v] : enumerate(Vs))
    {
        V.row(i) = v;
    }

    for(auto&& [i, f] : enumerate(Fs))
    {
        F.row(i) = f;
    }

    Eigen::MatrixXd V_new;
    Eigen::MatrixXi F_new;
    Eigen::VectorXi SVI, SVJ;
    igl::remove_duplicate_vertices(V, F, 1e-7, V_new, SVI, SVJ, F_new);

    vector<Vector3> Vs_new;
    Vs_new.resize(V_new.rows());
    for(auto&& [i, v] : enumerate(Vs_new))
    {
        v = V_new.row(i);
        apply_pre_transform(v);
    }

    vector<Vector3i> Fs_new;
    Fs_new.resize(F_new.rows());
    for(auto&& [i, f] : enumerate(Fs_new))
        f = F_new.row(i);

    return trimesh(Vs_new, Fs_new);
}


void SimplicialComplexIO::write(std::string_view file_name, const SimplicialComplex& sc)
{
    fs::path path{file_name};
    auto     ext = path.extension().string();
    // lowercase the extension
    std::ranges::transform(ext, ext.begin(), ::tolower);
    if(ext == ".obj")
    {
        write_obj(file_name, sc);
    }
    else if(ext == ".msh")
    {
        write_msh(file_name, sc);
    }
    else
    {
        throw GeometryIOError{fmt::format("Unsupported file format: {}", file_name)};
    }
}

void SimplicialComplexIO::write_obj(std::string_view file_name, const SimplicialComplex& sc)
{
    // NOTE: .obj file can only represent 0D, 1D and 2D facets
    // simplicial edge may be a subset of a simplicial triangle
    // here we exclude the simplicial edge that is not a facet.

    if(sc.dim() > 2)
    {
        throw GeometryIOError{fmt::format("Cannot write simplicial complex of dimension {} to .obj file",
                                          sc.dim())};
    }

    auto fp = std::fopen(file_name.data(), "w");

    if(!fp)
    {
        throw GeometryIOError{fmt::format("Failed to open file {} for writing.", file_name)};
    }

    span<const Vector3> Vs =
        sc.vertices().size() > 0 ? sc.positions().view() : span<const Vector3>{};

    if(Vs.size() == 0)
    {
        spdlog::warn("No vertices found in the simplicial complex. Writing an empty .obj file.");
    }

    auto Es = sc.edges().size() > 0 ? sc.edges().topo().view() : span<const Vector2i>{};
    auto Fs = sc.triangles().size() > 0 ? sc.triangles().topo().view() :
                                          span<const Vector3i>{};

    SizeT edge_count = 0;

    list<IndexT> surf_edge_indices;

    if(sc.dim() == 1)
        edge_count = sc.edges().size();
    else if(sc.dim() == 2)
    {
        auto is_facet = sc.edges().find<IndexT>(builtin::is_facet);
        if(is_facet)
        {
            auto is_facet_view = is_facet->view();
            for(auto&& [i, e] : enumerate(Es))
            {
                if(is_facet_view[i])
                    surf_edge_indices.push_back(i);
            }
        }
        else  // if no is_facet attribute, just write all edges
        {
            surf_edge_indices.resize(Es.size());
            std::iota(surf_edge_indices.begin(), surf_edge_indices.end(), 0);
        }

        edge_count = surf_edge_indices.size();
    }

    fmt::println(fp,
                 R"(#
# File generated by Libuipc
# Vertices {}
# Edges {}
# Faces {}
#)",
                 sc.vertices().size(),
                 edge_count,
                 sc.triangles().size());

    // write vertices
    for(auto&& v : Vs)
        fmt::println(fp, "v {} {} {}", v[0], v[1], v[2]);

    if(sc.dim() == 1)  // All edges are facets
    {
        // write edges
        for(auto&& e : Es)
            fmt::println(fp, "l {} {}", e[0] + 1, e[1] + 1);
    }
    else if(sc.dim() == 2)
    {
        // write edges
        for(auto&& i : surf_edge_indices)
        {
            auto e = Es[i];
            fmt::println(fp, "l {} {}", e[0] + 1, e[1] + 1);
        }

        // write faces
        auto orient = sc.triangles().find<IndexT>(builtin::orient);

        if(orient)
        {
            auto orient_view = sc.triangles().find<IndexT>(builtin::orient)->view();

            for(auto&& [i, F] : enumerate(Fs))
            {
                if(orient_view[i] >= 1)  // outward orientation, write as is
                    fmt::println(fp, "f {} {} {}", F[0] + 1, F[1] + 1, F[2] + 1);
                else  // inward orientation, flip the order
                    fmt::println(fp, "f {} {} {}", F[0] + 1, F[2] + 1, F[1] + 1);
            }
        }
        else
        {
            for(auto&& F : Fs)
                fmt::println(fp, "f {} {} {}", F[0] + 1, F[1] + 1, F[2] + 1);
        }
    }

    std::fclose(fp);
}


void SimplicialComplexIO::write_msh(std::string_view file_name, const SimplicialComplex& sc)
{
    const auto Dim = sc.dim();

    auto fp = std::fopen(file_name.data(), "w");

    if(!fp)
    {
        throw GeometryIOError{fmt::format("Failed to open file {} for writing.", file_name)};
    }

    auto Vs = sc.vertices().size() > 0 ? sc.positions().view() : span<const Vector3>{};

    if(Vs.size() == 0)
    {
        spdlog::warn("No vertices found in the simplicial complex. Writing an empty .msh file.");
    }

    fmt::println(fp,
                 R"($MeshFormat
2.2 0 8
$EndMeshFormat
)");

    fmt::println(fp, "$Nodes");
    fmt::println(fp, "{}", Vs.size());
    for(auto&& [i, v] : enumerate(Vs))
    {
        // ID x y z
        fmt::println(fp, "{} {} {} {}", i + 1, v[0], v[1], v[2]);
    }
    fmt::println(fp, "$EndNodes");

    fmt::println(fp, "$Elements");

    SizeT type     = Dim + 1;
    SizeT num_tags = 0;  // no tags

    if(Dim == 3)
    {
        auto Ts = sc.tetrahedra().size() > 0 ? sc.tetrahedra().topo().view() :
                                               span<const Vector4i>{};
        if(Ts.size() == 0)
        {
            spdlog::warn("No tetrahedra found in the simplicial complex. Writing only vertices.");
        }

        fmt::println(fp, "{}", Ts.size());
        for(auto&& [i, t] : enumerate(Ts))
        {
            // ID elm-type number-of-tags < tags ... > node-number-list

            fmt::println(fp,
                         "{} {} {} {} {} {} {}",
                         i + 1,
                         type,
                         num_tags,  //
                         t[0] + 1,
                         t[1] + 1,
                         t[2] + 1,
                         t[3] + 1);
        }
    }
    else if(Dim == 2)
    {
        auto Fs = sc.triangles().size() > 0 ? sc.triangles().topo().view() :
                                              span<const Vector3i>{};

        if(Fs.size() == 0)
        {
            spdlog::warn("No triangles found in the simplicial complex 2D. Writing only vertices.");
        }

        fmt::println(fp, "{}", Fs.size());
        for(auto&& [i, f] : enumerate(Fs))
        {
            // ID elm-type number-of-tags < tags ... > node-number-list

            fmt::println(fp,
                         "{} {} {} {} {} {}",
                         i + 1,
                         type,
                         num_tags,  //
                         f[0] + 1,
                         f[1] + 1,
                         f[2] + 1);
        }
    }
    else if(Dim == 1)
    {
        auto Es = sc.edges().size() > 0 ? sc.edges().topo().view() :
                                          span<const Vector2i>{};

        if(Es.size() == 0)
        {
            spdlog::warn("No edges found in the simplicial complex 1D. Writing only vertices.");
        }

        fmt::println(fp, "{}", Es.size());
        for(auto&& [i, e] : enumerate(Es))
        {
            // ID elm-type number-of-tags < tags ... > node-number-list

            fmt::println(fp,
                         "{} {} {} {} {}",
                         i + 1,
                         type,
                         num_tags,  //
                         e[0] + 1,
                         e[1] + 1);
        }
    }
    fmt::println(fp, "$EndElements");

    std::fclose(fp);
}
}  // namespace uipc::geometry
