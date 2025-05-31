#include <uipc/geometry/utils/tetrahedralize.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/builtin/attribute_name.h>
#include <tetgen/tetgen.h>
#include <iostream>

// ref: https://github.com/libigl/libigl/blob/main/include/igl/copyleft/tetgen/tetrahedralize.h
namespace uipc::geometry
{
// Load a vertex list and face list into a tetgenio object
tetgenio mesh_to_tetgenio(span<const Vector3>  V,   // vertex position list
                          span<const Vector3i> F,   // face list
                          span<const IndexT>   VM,  // vertex marker list
                          span<const IndexT>   FM,  // face marker list
                          span<const Vector3> H,  // seed point inside each hole
                          span<const Eigen::Vector<Float, 5>> R  // seed point inside each region
)
{
    using namespace std;

    tetgenio in;

    in.firstnumber    = 0;
    in.numberofpoints = V.size();
    in.pointlist      = new REAL[in.numberofpoints * 3];
    if(VM.size())
    {
        in.pointmarkerlist = new int[VM.size()];
    }
    //loop over points
    for(int i = 0; i < V.size(); i++)
    {
        in.pointlist[i * 3 + 0] = V[i].x();
        in.pointlist[i * 3 + 1] = V[i].y();
        in.pointlist[i * 3 + 2] = V[i].z();
        if(VM.size())
        {
            in.pointmarkerlist[i] = VM[i];
        }
    }
    in.numberoffacets  = F.size();
    in.facetlist       = new tetgenio::facet[in.numberoffacets];
    in.facetmarkerlist = new int[in.numberoffacets];

    // loop over face
    for(int i = 0; i < F.size(); i++)
    {
        in.facetmarkerlist[i] = FM.size() ? FM[i] : i;
        tetgenio::facet* f    = &in.facetlist[i];
        f->numberofpolygons   = 1;
        f->polygonlist        = new tetgenio::polygon[f->numberofpolygons];
        f->numberofholes      = 0;
        f->holelist           = NULL;
        tetgenio::polygon* p  = &f->polygonlist[0];
        p->numberofvertices   = 3;  // only triangles
        p->vertexlist         = new int[p->numberofvertices];
        // loop around face
        for(int j = 0; j < p->numberofvertices; j++)
        {
            p->vertexlist[j] = F[i][j];
        }
    }

    in.numberofholes = H.size();
    in.holelist      = new REAL[3 * in.numberofholes];
    // loop over holes
    for(int holeID = 0; holeID < H.size(); holeID++)
    {
        in.holelist[holeID * 3 + 0] = H[holeID].x();
        in.holelist[holeID * 3 + 1] = H[holeID].y();
        in.holelist[holeID * 3 + 2] = H[holeID].z();
    }

    in.numberofregions = R.size();
    in.regionlist      = new REAL[5 * in.numberofregions];
    // loop over regions
    for(int regionID = 0; regionID < R.size(); regionID++)
    {
        in.regionlist[regionID * 5 + 0] = R[regionID][0];
        in.regionlist[regionID * 5 + 1] = R[regionID][1];
        in.regionlist[regionID * 5 + 2] = R[regionID][2];
        in.regionlist[regionID * 5 + 3] = R[regionID][3];
        in.regionlist[regionID * 5 + 4] = R[regionID][4];
    }

    return in;
}

/// Convert a tetgenio to a tetmesh
///
/// @param[in] out output of tetrahedralization
/// @param[out] V  #V by 3 list of mesh vertex positions
/// @param[out] T  #T by 4 list of mesh tet indices into V
/// @param[out] F  #F by 3 list of mesh triangle indices into V
/// @param[out] TM  #T by 1 list of material indices into R
/// @param[out] R  #TT list of region ID for each tetrahedron
/// @param[out] N  #TT by 4 list of indices neighbors for each tetrahedron ('n')
/// @param[out] PT  #TV list of incident tetrahedron for a vertex ('m')
/// @param[out] FT  #TF by 2 list of tetrahedrons sharing a triface ('nn')
/// @param[out] num_regions Number of regions in output mesh
///
/// \bug Assumes that out.numberoftetrahedronattributes == 1 or 0
template <typename DerivedV, typename DerivedT, typename DerivedF, typename DerivedTM, typename DerivedR, typename DerivedN, typename DerivedPT, typename DerivedFT>
bool tetgenio_to_tetmesh(const tetgenio&                    out,
                         Eigen::PlainObjectBase<DerivedV>&  V,
                         Eigen::PlainObjectBase<DerivedT>&  T,
                         Eigen::PlainObjectBase<DerivedF>&  F,
                         Eigen::PlainObjectBase<DerivedTM>& TM,
                         Eigen::PlainObjectBase<DerivedR>&  R,
                         Eigen::PlainObjectBase<DerivedN>&  N,
                         Eigen::PlainObjectBase<DerivedPT>& PT,
                         Eigen::PlainObjectBase<DerivedFT>& FT,
                         int&                               num_regions)
{
    // process points
    if(out.pointlist == NULL)
    {
        UIPC_ASSERT(false, "tetgenio_to_tetmesh Error: point list is NULL");
        return false;
    }
    V.resize(out.numberofpoints, 3);
    // loop over points
    for(int i = 0; i < out.numberofpoints; i++)
    {
        V(i, 0) = out.pointlist[i * 3 + 0];
        V(i, 1) = out.pointlist[i * 3 + 1];
        V(i, 2) = out.pointlist[i * 3 + 2];
    }

    // process tets
    if(out.tetrahedronlist == NULL)
    {
        UIPC_ASSERT(false, "tetgenio_to_tetmesh Error: tet list is NULL");
        return false;
    }

    // When would this not be 4?
    UIPC_ASSERT(out.numberofcorners == 4, "tetgenio_to_tetmesh Error: number of corners is not 4");
    T.resize(out.numberoftetrahedra, out.numberofcorners);
    // loop over tetrahedra
    for(int i = 0; i < out.numberoftetrahedra; i++)
    {
        for(int j = 0; j < out.numberofcorners; j++)
        {
            T(i, j) = out.tetrahedronlist[i * out.numberofcorners + j];
        }
    }

    F.resize(out.numberoftrifaces, 3);
    // loop over tetrahedra
    for(int i = 0; i < out.numberoftrifaces; i++)
    {
        F(i, 0) = out.trifacelist[i * 3 + 0];
        F(i, 1) = out.trifacelist[i * 3 + 1];
        F(i, 2) = out.trifacelist[i * 3 + 2];
    }

    if(out.pointmarkerlist)
    {
        TM.resize(out.numberofpoints);
        for(int i = 0; i < out.numberofpoints; ++i)
        {
            TM(i) = out.pointmarkerlist[i];
        }
    }

    if(out.tetrahedronattributelist)
    {
        R.resize(out.numberoftetrahedra);
        std::unordered_map<REAL, REAL> hashUniqueRegions;
        for(int i = 0; i < out.numberoftetrahedra; i++)
        {
            R(i)                    = out.tetrahedronattributelist[i];
            hashUniqueRegions[R(i)] = i;
        }
        // extract region marks
        num_regions = hashUniqueRegions.size();
    }
    else
    {
        num_regions = 0;
    }

    // extract neighbor list
    if(out.neighborlist)
    {
        N.resize(out.numberoftetrahedra, 4);
        for(int i = 0; i < out.numberoftetrahedra; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                N(i, j) = out.neighborlist[i * 4 + j];
            }
        }
    }

    // extract point 2 tetrahedron list
    if(out.point2tetlist)
    {
        PT.resize(out.numberofpoints);
        for(int i = 0; i < out.numberofpoints; i++)
        {
            PT(i) = out.point2tetlist[i];
        }
    }

    //extract face to tetrahedron list
    if(out.face2tetlist)
    {
        FT.resize(out.numberoftrifaces, 2);
        int triface;
        for(int i = 0; i < out.numberoftrifaces; i++)
        {
            for(int j = 0; j < 2; j++)
            {
                FT(i, j) = out.face2tetlist[i * 2 + j];
            }
        }
    }

    return true;
}

SimplicialComplex tetrahedralize(const SimplicialComplex& sc, const Json& options)
{
    // REAL is what tetgen is using as floating point precision
    // Prepare input


    auto V = sc.positions().view();
    auto F = sc.triangles().topo().view();

    tetgenio in = mesh_to_tetgenio(V, F, {}, {}, {}, {});

    std::string switches;
    if(options.is_object())
    {
        if(options.contains("switches"))
        {
            switches = options["switches"].get<std::string>();
        }
    }

    tetgenio       out;
    tetgenbehavior b;
    ::tetrahedralize(&b, &in, &out);

    if(out.numberoftetrahedra == 0)
    {
        UIPC_WARN_WITH_LOCATION("No tetrahedra generated. Empty simplicial complex returned");
        return SimplicialComplex{};
    }

    Eigen::MatrixX<Float> TV;  // vertex position list
    Eigen::MatrixXi       TT;  // tetrahedron list
    Eigen::MatrixXi       TF;  // triangle list
    Eigen::VectorXi       TM;  // material list
    Eigen::VectorXi       TR;  // region list
    Eigen::MatrixXi       TN;  // neighbor list
    Eigen::VectorXi       PT;  // point to tetrahedron list
    Eigen::MatrixXi       FT;  // face to tetrahedron list
    int                   num_regions;

    // Prepare output
    bool ret = tetgenio_to_tetmesh(out, TV, TT, TF, TM, TR, TN, PT, FT, num_regions);
    if(!ret)
    {
        UIPC_WARN_WITH_LOCATION("Tetrahedralization failed. Empty simplicial complex returned. Message");
        return SimplicialComplex{};
    }

    SimplicialComplex R;
    {
        R.vertices().resize(TV.rows());
        auto pos = R.vertices().create<Vector3>(builtin::position, Vector3::Zero(), false);
        auto pos_view = view(*pos);

        for(int i = 0; i < TV.rows(); i++)
        {
            pos_view[i] = TV.row(i).segment<3>(0);
        }

        R.tetrahedra().resize(TT.rows());
        auto topo = R.tetrahedra().create<Vector4i>(builtin::topo, Vector4i::Zero(), false);
        auto topo_view = view(*topo);

        for(int i = 0; i < TT.rows(); i++)
        {
            topo_view[i] = TT.row(i).segment<4>(0);
        }
    }

    return facet_closure(R);
}
}  // namespace uipc::geometry
