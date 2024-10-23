#pragma once
#include <uipc/common/macro.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/vector.h>
#include <uipc/geometry/attribute_slot.h>
namespace uipc::core
{
class DiffSim;
}

namespace uipc::diff_sim
{
class Object;
class ParameterCollection;

span<Float> UIPC_CORE_API view(ParameterCollection& collection);

class UIPC_CORE_API ParameterCollection
{
  public:
    void resize(SizeT N, Float default_value = 0.0f);
    /**
     * @brief Broadcast the parameter collection to corresponding geometry attributes.
     * 
     */
    void              broadcast();
    span<const Float> view() const;

  private:
    friend class uipc::core::DiffSim;

    ParameterCollection();
    ~ParameterCollection();

    // delete copy constructor and assignment operator
    ParameterCollection(const ParameterCollection&)            = delete;
    ParameterCollection& operator=(const ParameterCollection&) = delete;

    class Impl;
    friend span<Float> UIPC_CORE_API view(ParameterCollection& collection);
    void connect(S<geometry::IAttributeSlot> diff_parm_slot,
                 S<geometry::IAttributeSlot> parm_slot);
    void build();

    U<Impl> m_impl;
};
}  // namespace uipc::diff_sim
