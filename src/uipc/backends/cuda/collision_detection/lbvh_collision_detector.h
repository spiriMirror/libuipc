#pragma once
#include <sim_system.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/atomic_counting_lbvh.h>

namespace uipc::backend::cuda
{
class LBVHCollisionDetector : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        void detect_candidates(GlobalCollisionDetector::DetectCandidateInfo& info);

        GlobalVertexManager*  global_vertex_manager;
        GlobalSurfaceManager* global_surface_manager;
        GlobalContactManager* global_contact_manager;


        muda::DeviceBuffer<AABB>     point_aabbs;
        muda::DeviceBuffer<AABB>     triangle_aabbs;
        AtomicCountingLBVH           lbvh_PT;
        muda::DeviceBuffer<Vector4i> candidate_PTs;

        muda::DeviceBuffer<AABB>     edge_aabbs;
        AtomicCountingLBVH           lbvh_EE;
        muda::DeviceBuffer<Vector4i> candidate_EEs;
    };

  protected:
    virtual void do_build() override;

  private:
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
