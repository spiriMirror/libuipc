#include <collision_detection/global_collision_detector.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <collision_detection/collision_detector.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<GlobalCollisionDetector>
{
  public:
    static U<GlobalCollisionDetector> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<GlobalCollisionDetector>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(GlobalCollisionDetector);

void GlobalCollisionDetector::detect_candidates(Float alpha)
{
    DetectCandidateInfo info;
    info.m_alpha = alpha;
    UIPC_ASSERT(m_impl.detect_candidates != nullptr, "Collision detector is not set.");
    m_impl.detect_candidates(info);
    m_impl.candidate_PTs = info.m_candidate_PTs;
    m_impl.candidate_EEs = info.m_candidate_EEs;
}

void GlobalCollisionDetector::classify_candidates()
{
    ClassifyCandidateInfo info;

    info.m_candidate_PTs = m_impl.candidate_PTs;
    info.m_candidate_EEs = m_impl.candidate_EEs;

    UIPC_ASSERT(m_impl.classify_candidates != nullptr, "Collision detector is not set.");

    m_impl.classify_candidates(info);

    m_impl.PTs = info.m_PTs;
    m_impl.EEs = info.m_EEs;
    m_impl.PEs = info.m_PEs;
    m_impl.PPs = info.m_PPs;
}

void GlobalCollisionDetector::add_detector(std::function<void(DetectCandidateInfo&)>&& detector)
{
    check_state(SimEngineState::BuildSystems, "add_detector()");

    UIPC_ASSERT(m_impl.detect_candidates == nullptr, "More than one collision detector is added");

    m_impl.detect_candidates = std::move(detector);
}
void GlobalCollisionDetector::add_classifier(std::function<void(ClassifyCandidateInfo&)>&& classifier)
{
    check_state(SimEngineState::BuildSystems, "add_classifier()");

    UIPC_ASSERT(m_impl.classify_candidates == nullptr,
                "More than one collision classifier is added");

    m_impl.classify_candidates = std::move(classifier);
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::candidate_PTs() const
{
    return m_impl.candidate_PTs;
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::candidate_EEs() const
{
    return m_impl.candidate_EEs;
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::PTs() const
{
    return m_impl.PTs;
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::EEs() const
{
    return m_impl.EEs;
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::PEs() const
{
    return m_impl.PEs;
}

muda::CBufferView<Vector4i> GlobalCollisionDetector::PPs() const
{
    return m_impl.PPs;
}
}  // namespace uipc::backend::cuda