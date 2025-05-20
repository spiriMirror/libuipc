#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/diff_sim/parameter_collection.h>
#include <uipc/diff_sim/sparse_coo_view.h>

namespace uipc::backend
{
class SceneVisitor;
class DiffSimVisitor;
}  // namespace uipc::backend

namespace uipc::core::internal
{
class Scene;
}

namespace uipc::core
{
class Scene;
class UIPC_CORE_API DiffSim
{
    friend class Scene;
    friend class internal::Scene;
    friend class backend::DiffSimVisitor;


    DiffSim();

    // delete copy constructor and assignment operator
    DiffSim(const DiffSim&)            = delete;
    DiffSim& operator=(const DiffSim&) = delete;

  public:
    diff_sim::ParameterCollection&       parameters();
    const diff_sim::ParameterCollection& parameters() const;
    ~DiffSim();

  private:
    class Impl;
    U<Impl> m_impl;

    void init(internal::Scene& scene);  // only be called by Scene
};
}  // namespace uipc::core
