#pragma once
#include <uipc/common/macro.h>
#include <uipc/common/span.h>
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/vector.h>

namespace uipc::diff_sim
{
class Object;
class DiffSim;

class UIPC_CORE_API ParameterCollection
{
    friend class DiffSim;

    ParameterCollection();
    ~ParameterCollection();

    // delete copy constructor and assignment operator
    ParameterCollection(const ParameterCollection&)            = delete;
    ParameterCollection& operator=(const ParameterCollection&) = delete;

    class Impl;
    friend span<Float> view(ParameterCollection& collection);

  public:
    void              resize(SizeT N, Float default_value = 0.0f);
    span<const Float> view() const;
    U<Impl>           m_impl;
};

span<Float> UIPC_CORE_API view(ParameterCollection& collection);
}  // namespace uipc::diff_sim
