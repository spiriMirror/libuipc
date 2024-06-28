#include <collision_detection/collision_classifier.h>
#include <sim_engine.h>
#include <muda/ext/geo/distance.h>
#include <muda/cub/device/device_select.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<CollisionClassifier>
{
  public:
    static U<CollisionClassifier> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<CollisionClassifier>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(CollisionClassifier);

void CollisionClassifier::do_build()
{
    auto global_collision_detector = find<GlobalCollisionDetector>();
    global_collision_detector->add_classifier(
        [this](GlobalCollisionDetector::ClassifyCandidateInfo& info)
        { m_impl.classify_candidates(info); });

    m_impl.global_vertex_manager  = find<GlobalVertexManager>();
    m_impl.global_contact_manager = find<GlobalContactManager>();
}

void CollisionClassifier::Impl::classify_candidates(GlobalCollisionDetector::ClassifyCandidateInfo& info)
{
    using namespace muda;
    auto d_hat         = global_contact_manager->d_hat();
    auto Ps            = global_vertex_manager->positions();
    auto candidate_PTs = info.candidate_PTs();
    auto candidate_EEs = info.candidate_EEs();

    temp_PTs.resize(candidate_PTs.size(), Vector4i::Ones() * -1);
    PTs.resize(candidate_PTs.size());
    temp_EEs.resize(candidate_EEs.size(), Vector4i::Ones() * -1);
    EEs.resize(candidate_EEs.size());

    // PE:
    SizeT max_PE_count = candidate_PTs.size() + candidate_EEs.size();
    temp_PEs.resize(max_PE_count, Vector3i::Ones() * -1);
    PEs.resize(max_PE_count);
    // PP:
    SizeT max_PP_count = candidate_PTs.size() + candidate_EEs.size();
    temp_PPs.resize(max_PP_count, Vector2i::Ones() * -1);
    PPs.resize(max_PP_count);  // reserve enough space for PTs


    auto PE_offset = 0;
    auto PE_count  = candidate_PTs.size();
    auto PP_offset = 0;
    auto PP_count  = candidate_PTs.size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_PTs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                PTs           = temp_PTs.viewer().name("PTs"),
                PEs = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs")] __device__(int i) mutable
               {
                   Vector4i PT = candidate_PTs(i);
                   IndexT   P  = PT(0);
                   Vector3  T  = PT.tail<3>();


                   auto V  = Ps(PT(0));
                   auto F0 = Ps(PT(1));
                   auto F1 = Ps(PT(2));
                   auto F2 = Ps(PT(3));

                   auto dist_type = distance::point_triangle_distance_type(V, F0, F1, F2);

                   switch(dist_type)
                   {
                       case muda::distance::PointTriangleDistanceType::PP_PT0:
                           PPs(i) = Vector2i(P, T(0));
                           break;
                       case muda::distance::PointTriangleDistanceType::PP_PT1:
                           PPs(i) = Vector2i(P, T(1));
                           break;
                       case muda::distance::PointTriangleDistanceType::PP_PT2:
                           PPs(i) = Vector2i(P, T(2));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT0T1:
                           PEs(i) = Vector3i(P, T(0), T(1));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT1T2:
                           PEs(i) = Vector3i(P, T(1), T(2));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT2T0:
                           PEs(i) = Vector3i(P, T(2), T(0));
                           break;
                       case muda::distance::PointTriangleDistanceType::PT:
                           PTs(i) = PT;
                           break;
                       default:
                           break;
                   }
               });

    PE_offset = PE_count;
    PE_count  = candidate_EEs.size();
    PP_offset = PP_count;
    PP_count  = candidate_EEs.size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_EEs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                EEs           = temp_EEs.viewer().name("EEs"),
                PEs = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs")] __device__(int i) mutable
               {
                   Vector4i EE = candidate_EEs(i);

                   IndexT Ea0 = EE(0);
                   IndexT Ea1 = EE(1);
                   IndexT Eb0 = EE(2);
                   IndexT Eb1 = EE(3);

                   Vector3 E0 = Ps(Ea0);
                   Vector3 E1 = Ps(Ea1);
                   Vector3 E2 = Ps(Eb0);
                   Vector3 E3 = Ps(Eb1);

                   auto dist_type = distance::edge_edge_distance_type(E0, E1, E2, E3);

                   switch(dist_type)
                   {
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea0Eb0:
                           PPs(i) = Vector2i(Ea0, Eb0);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea0Eb1:
                           PPs(i) = Vector2i(Ea0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Ea0Eb0Eb1:
                           PEs(i) = Vector3i(Ea0, Eb0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea1Eb0:
                           PPs(i) = Vector2i(Ea1, Eb0);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea1Eb1:
                           PPs(i) = Vector2i(Ea1, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Ea1Eb0Eb1:
                           PEs(i) = Vector3i(Ea1, Eb0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Eb0Ea0Ea1:
                           PEs(i) = Vector3i(Eb0, Ea0, Ea1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Eb1Ea0Ea1:
                           PEs(i) = Vector3i(Eb1, Ea0, Ea1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::EE:
                           EEs(i) = EE;
                           break;
                       default:
                           break;
                   }
               });

    DeviceSelect().If(temp_PTs.data(),
                      PTs.data(),
                      selected.data(),
                      temp_PTs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector4i& pt)
                      { return pt(0) != -1; });

    int h_selected = selected;
    PTs.resize(h_selected);

    DeviceSelect().If(temp_EEs.data(),
                      EEs.data(),
                      selected.data(),
                      temp_EEs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector4i& ee)
                      { return ee(0) != -1; });

    h_selected = selected;
    EEs.resize(h_selected);

    DeviceSelect().If(temp_PEs.data(),
                      PEs.data(),
                      selected.data(),
                      temp_PEs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector3i& pe)
                      { return pe(0) != -1; });

    h_selected = selected;
    PEs.resize(h_selected);

    DeviceSelect().If(temp_PPs.data(),
                      PPs.data(),
                      selected.data(),
                      temp_PPs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector2i& pp)
                      { return pp(0) != -1; });

    h_selected = selected;
    PPs.resize(h_selected);

    info.PTs(PTs);
    info.EEs(EEs);
    info.PEs(PEs);
    info.PPs(PPs);
}
}  // namespace uipc::backend::cuda
