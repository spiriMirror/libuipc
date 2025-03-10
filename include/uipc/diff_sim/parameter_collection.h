#pragma once
#include <uipc/common/dllexport.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/vector.h>
#include <uipc/geometry/attribute_slot.h>
namespace uipc::core
{
class DiffSim;
}

namespace uipc::backend
{
class DiffSimVisitor;
}

namespace uipc::diff_sim
{
class Object;
class ParameterCollection;

UIPC_CORE_API span<Float> view(ParameterCollection& collection);
UIPC_CORE_API Eigen::Map<Eigen::Matrix<Float, Eigen::Dynamic, 1>> as_eigen(ParameterCollection& collection);

class UIPC_CORE_API ParameterCollection
{
  public:
    void resize(SizeT N, Float default_value = 0.0f);
    /**
     * @brief Broadcast the parameter collection to corresponding geometry attributes.
     * 
     */
    void                                                      broadcast();
    span<const Float>                                         view() const;
    Eigen::Map<const Eigen::Matrix<Float, Eigen::Dynamic, 1>> as_eigen() const;
    SizeT                                                     size() const;

    ~ParameterCollection();

  private:
    friend class uipc::core::DiffSim;
    friend class uipc::backend::DiffSimVisitor;

    ParameterCollection();

    // delete copy constructor and assignment operator
    ParameterCollection(const ParameterCollection&)            = delete;
    ParameterCollection& operator=(const ParameterCollection&) = delete;

    class Impl;
    friend UIPC_CORE_API span<Float> view(ParameterCollection& collection);
    friend UIPC_CORE_API Eigen::Map<Eigen::Matrix<Float, Eigen::Dynamic, 1>> as_eigen(span<Float> s);
    void connect(S<geometry::IAttributeSlot> diff_parm_slot,
                 S<geometry::IAttributeSlot> parm_slot);
    void build();

    bool    need_backend_broadcast() const;
    void    need_backend_broadcast(bool v);
    U<Impl> m_impl;
};
}  // namespace uipc::diff_sim
