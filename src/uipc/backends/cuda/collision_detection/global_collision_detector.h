#pragma once
#include <sim_system.h>
#include <collision_detection/linear_bvh.h>

namespace uipc::backend::cuda
{
class GlobalContactManager;
class GlobalVertexManager;
class GlobalSurfaceManager;

class GlobalCollisionDetector : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl;

    class Impl
    {
      public:
        void detect_candidates();
        void detect_candidates(Float alpha);


        GlobalContactManager* global_contact_manager = nullptr;
        GlobalVertexManager*  global_vertex_manager  = nullptr;
        GlobalSurfaceManager* global_surface_manager = nullptr;

        muda::DeviceBuffer<Vector4i> candidate_PTs;  // candidate-point-triangle pairs
        muda::DeviceBuffer<Vector4i> candidate_EEs;  // candidate-edge-edge pairs

        muda::DeviceBuffer<Vector4i> classified_PTs;  // exact-point-triangle pairs
        muda::DeviceBuffer<Vector4i> classified_EEs;  // exact-edge-edge pairs
        muda::DeviceBuffer<Vector3i> classified_PEs;  // exact-point-edge pairs
        muda::DeviceBuffer<Vector2i> classified_PPs;  // exact-point-point pairs

        // TODO: now we just hard-code the implementation
        // later we will create an interface class for collision detection methods
        LinearBVH bvh;
    };

  protected:
    virtual void do_build() override;

  private:
    friend class SimEngine;
    Impl m_impl;

    void detect_candidates();
    void detect_candidates(Float alpha);
};
}  // namespace uipc::backend::cuda