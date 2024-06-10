#include <uipc/geometry/utils/closure.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>

namespace uipc::geometry
{
static void facet_closure_dim_2(SimplicialComplex& R)
{
    /*
    * generate the edges from the triangles
    */

    // try to find the unique edges from the faces
    auto F = R.triangles().topo().view();

    // the edges of the faces
    vector<Vector2i> sep_edges(F.size() * 3);

    // make sure the edge is sorted
    auto sort_edge = [](Vector2i e)
    {
        std::ranges::sort(e);
        return e;
    };


    // set the edges
    for(auto [i, f] : enumerate(F))
    {
        sep_edges[i * 3]     = sort_edge({f[0], f[1]});
        sep_edges[i * 3 + 1] = sort_edge({f[1], f[2]});
        sep_edges[i * 3 + 2] = sort_edge({f[2], f[0]});
    }

    // sort the edges
    std::ranges::sort(sep_edges,
                      [](Vector2i a, Vector2i b)
                      { return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1]); });

    // make unique and erase the duplicate edges
    auto [first, last] = std::ranges::unique(sep_edges);
    sep_edges.erase(first, last);

    // now we have the unique edges
    R.edges().resize(sep_edges.size());

    // copy_from the edges to the new complex
    auto edge_view = view(R.edges().topo());
    std::ranges::copy(sep_edges, edge_view.begin());
}

static void facet_closure_dim_3(SimplicialComplex& R)
{
    // try to find the unique faces from the tetrahedra
    auto T = R.tetrahedra().topo().view();

    // the faces of the tetrahedra
    vector<Vector3i> sep_faces(T.size() * 4);

    // make sure the face is sorted
    auto sort_face = [](Vector3i f)
    {
        std::ranges::sort(f);
        return f;
    };

    // set the faces
    for(auto [i, t] : enumerate(T))
    {
        sep_faces[i * 4 + 0] = sort_face({t[0], t[1], t[2]});
        sep_faces[i * 4 + 1] = sort_face({t[0], t[1], t[3]});
        sep_faces[i * 4 + 2] = sort_face({t[0], t[2], t[3]});
        sep_faces[i * 4 + 3] = sort_face({t[1], t[2], t[3]});
    }

    // sort the faces
    std::ranges::sort(sep_faces,
                      [](Vector3i a, Vector3i b)
                      {
                          return a[0] < b[0] || (a[0] == b[0] && a[1] < b[1])
                                 || (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
                      });

    // make unique and erase the duplicate faces
    auto [first, last] = std::ranges::unique(sep_faces);
    sep_faces.erase(first, last);

    // now we have the unique faces
    R.triangles().resize(sep_faces.size());

    // copy_from the faces to the new complex
    auto face_view = view(R.triangles().topo());
    std::ranges::copy(sep_faces, face_view.begin());

    // then we use the triangles to generate the edges
    facet_closure_dim_2(R);
}

static void check_facet_closure_input(const SimplicialComplex& O)
{
    UIPC_ASSERT(O.dim() >= 0 && O.dim() <= 3,
                "When calling `facet_closure()`, your simplicial complex should be in dimension [0, 3], your dimension ({}).",
                O.dim());

    switch(O.dim())
    {
        case 2: {
            UIPC_ASSERT(O.edges().size() == 0,
                        "When calling `facet_closure()`, your lower dimensional simplicial should be empty, your edge count ({}).",
                        O.edges().size());
        }
        break;
        case 3: {
            UIPC_ASSERT(O.triangles().size() == 0 && O.edges().size() == 0,
                        "When calling `facet_closure()`, your lower dimensional simplicial should be empty, your face count ({}), yout edge count ({}).",
                        O.triangles().size(),
                        O.edges().size());
        }
        break;
        default:
            break;
    }
}

SimplicialComplex facet_closure(const SimplicialComplex& O)
{
    check_facet_closure_input(O);

    SimplicialComplex R;

    // share vertices
    R.vertices().resize(O.vertices().size());
    R.vertices().topo().share(O.vertices().topo());
    R.vertices().share(builtin::position, O.positions());

    switch(O.dim())
    {
        case 0: {
            // nothing to do, vertices are closure of vertices
        }
        break;
        case 1: {
            // share edges
            R.edges().resize(O.edges().size());
            R.edges().topo().share(O.edges().topo());
            // no need to do anything, edges and vertices are closure of edges
        }
        break;
        case 2: {
            // share triangles
            R.triangles().resize(O.triangles().size());
            R.triangles().topo().share(O.triangles().topo());
            // generate the edges from the triangles
            facet_closure_dim_2(R);
        }
        break;
        case 3: {
            // share tetrahedra
            R.tetrahedra().resize(O.tetrahedra().size());
            R.tetrahedra().topo().share(O.tetrahedra().topo());
            // generate the faces from the tetrahedrad
            facet_closure_dim_3(R);
        }
        break;
        default:
            break;
    }

    return R;
}
}  // namespace uipc::geometry
