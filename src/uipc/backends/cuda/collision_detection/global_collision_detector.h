#pragma once
#include <sim_system.h>
#include <collision_detection/linear_bvh.h>

namespace uipc::backend::cuda
{
class CollisionDetector;

class GlobalCollisionDetector : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;


    class DetectCandidateInfo
    {
      public:
        Float alpha() const { return m_alpha; }
        void  candidate_PTs(muda::CBufferView<Vector4i> candidate_PTs)
        {
            m_candidate_PTs = candidate_PTs;
        }
        void candidate_EEs(muda::CBufferView<Vector4i> candidate_EEs)
        {
            m_candidate_EEs = candidate_EEs;
        }

      private:
        friend class GlobalCollisionDetector;
        Float                       m_alpha = 0.0;
        muda::CBufferView<Vector4i> m_candidate_PTs;
        muda::CBufferView<Vector4i> m_candidate_EEs;
    };

    class ClassifyCandidateInfo
    {
      public:
        muda::CBufferView<Vector4i> candidate_PTs() const
        {
            return m_candidate_PTs;
        }
        muda::CBufferView<Vector4i> candidate_EEs() const
        {
            return m_candidate_EEs;
        }


        void PTs(muda::CBufferView<Vector4i> PTs) { m_PTs = PTs; }
        void EEs(muda::CBufferView<Vector4i> EEs) { m_EEs = EEs; }
        void PEs(muda::CBufferView<Vector3i> PEs) { m_PEs = PEs; }
        void PPs(muda::CBufferView<Vector2i> PPs) { m_PPs = PPs; }


      private:
        friend class GlobalCollisionDetector;

        muda::CBufferView<Vector4i> m_candidate_PTs;
        muda::CBufferView<Vector4i> m_candidate_EEs;

        muda::CBufferView<Vector4i> m_PTs;
        muda::CBufferView<Vector4i> m_EEs;
        muda::CBufferView<Vector3i> m_PEs;
        muda::CBufferView<Vector2i> m_PPs;
    };

    class Impl
    {
      public:
        std::function<void(DetectCandidateInfo&)> detect_candidates = nullptr;
        std::function<void(ClassifyCandidateInfo&)> classify_candidates = nullptr;

        muda::CBufferView<Vector4i> candidate_PTs;
        muda::CBufferView<Vector4i> candidate_EEs;

        muda::CBufferView<Vector4i> PTs;
        muda::CBufferView<Vector4i> EEs;
        muda::CBufferView<Vector3i> PEs;
        muda::CBufferView<Vector2i> PPs;
    };


    muda::CBufferView<Vector4i> candidate_PTs() const;  // candidate-point-triangle m_pairs
    muda::CBufferView<Vector4i> candidate_EEs() const;  // candidate-edge-edge m_pairs

    muda::CBufferView<Vector4i> PTs() const;  // point-triangle m_pairs
    muda::CBufferView<Vector4i> EEs() const;  // edge-edge m_pairs
    muda::CBufferView<Vector4i> PEs() const;  // point-edge m_pairs
    muda::CBufferView<Vector4i> PPs() const;  // point-point m_pairs

    void add_detector(std::function<void(DetectCandidateInfo&)>&& detector);
    void add_classifier(std::function<void(ClassifyCandidateInfo&)>&& classifier);

  protected:
  private:
    friend class SimEngine;

    void detect_candidates(Float alpha);
    void classify_candidates();

    Impl m_impl;
};
}  // namespace uipc::backend::cuda