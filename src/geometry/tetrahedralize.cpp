#include <uipc/geometry/utils/tetrahedralize.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/builtin/attribute_name.h>
#include <ftetwild/ftetwild.h>

namespace uipc::geometry
{
SimplicialComplex tetrahedralize(const SimplicialComplex& sc,
                                 Float       ideal_edge_length_ratio,
                                 const Json& options)
{
    ftetwild::Parameters params;
    params.ideal_edge_length_rel(ideal_edge_length_ratio);
    params.log_level(static_cast<int>(logger::get_level()));

    ftetwild::FtetWild tetwild;

    Eigen::MatrixXd InputVs;
    Eigen::MatrixXi InputFs;

    InputVs.resize(sc.vertices().size(), 3);
    auto v_span = sc.positions().view();
    for(auto&& [i, v] : enumerate(v_span))
        InputVs.row(i) = v;  // x,y,z

    auto f_span = sc.triangles().topo().view();
    InputFs.resize(sc.triangles().size(), 3);
    for(auto&& [i, f] : enumerate(f_span))
        InputFs.row(i) = f;  // v0,v1,v2

    Eigen::MatrixXd OuputVs;
    Eigen::MatrixXi OuputTs;

    tetwild.tetrahedralization(InputVs, InputFs, OuputVs, OuputTs, params);

    SimplicialComplex R;
    // create topo
    R.tetrahedra().resize(OuputTs.rows());
    auto topo = R.tetrahedra().create<Vector4i>(builtin::topo, Vector4i::Zero(), false);
    auto topo_view = view(*topo);
    for(auto&& [i, t] : enumerate(topo_view))
        t = OuputTs.row(i);  // v0,v1,v2,v3
    // create vertices
    R.vertices().resize(OuputVs.rows());
    auto pos = R.vertices().create<Vector3>(builtin::position, Vector3::Zero(), false);
    auto pos_view = view(*pos);
    for(auto&& [i, v] : enumerate(pos_view))
        v = OuputVs.row(i);  // x,y,z

    return facet_closure(R);  // make standard simplicial complex
}
}  // namespace uipc::geometry
