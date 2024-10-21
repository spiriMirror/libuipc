#pragma once
#include <uipc/common/macro.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/vector.h>
#include <uipc/diff_sim/parameter_bundle.h>

namespace uipc::diff_sim
{
class Object;
class DiffSim;
class UIPC_CORE_API ParameterCollection
{
    friend class DiffSim;

    ParameterCollection()  = default;
    ~ParameterCollection() = default;

    // delete copy constructor and assignment operator
    ParameterCollection(const ParameterCollection&)            = delete;
    ParameterCollection& operator=(const ParameterCollection&) = delete;

  public:
    S<ParameterBundle> create(std::string_view name, SizeT N, Float default_value = 0.0f);
};
}  // namespace uipc::diff_sim
