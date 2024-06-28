#pragma once
#include <sim_system.h>
#include <collision_detection/global_collision_detector.h>
#include <global_geometry/global_vertex_manager.h>
#include <contact_system/global_contact_manager.h>

namespace uipc::backend::cuda
{
class CollisionClassifier : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        void classify_candidates(GlobalCollisionDetector::ClassifyCandidateInfo& info);

        GlobalVertexManager*  global_vertex_manager  = nullptr;
        GlobalContactManager* global_contact_manager = nullptr;


        muda::DeviceBuffer<Vector4i> temp_PTs;
        muda::DeviceBuffer<Vector4i> temp_EEs;
        muda::DeviceBuffer<Vector3i> temp_PEs;
        muda::DeviceBuffer<Vector2i> temp_PPs;

        muda::DeviceBuffer<Vector4i> PTs;
        muda::DeviceBuffer<Vector4i> EEs;
        muda::DeviceBuffer<Vector3i> PEs;
        muda::DeviceBuffer<Vector2i> PPs;

        muda::DeviceVar<IndexT> selected;
    };


  protected:
    virtual void do_build() override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
