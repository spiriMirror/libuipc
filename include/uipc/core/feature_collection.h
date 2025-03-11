#pragma once
#include <uipc/common/unordered_map.h>
#include <uipc/common/smart_pointer.h>
#include <uipc/core/feature.h>
#include <uipc/common/json.h>

namespace uipc::core
{
class UIPC_CORE_API FeatureCollection final
{
  public:
    virtual ~FeatureCollection() = default;
    S<IFeature> find(std::string_view name) const;

    template <std::derived_from<IFeature> T>
    S<T> find(std::string_view name = T::FeatureName) const
    {
        return std::dynamic_pointer_cast<T>(find(name));
    }

    void insert(std::string_view name, S<IFeature> feature);

    template <std::derived_from<IFeature> T>
    void insert(S<T> feature)
    {
        insert(T::FeatureName, feature);
    }

    Json to_json() const;

  private:
    unordered_map<string, S<IFeature>> m_features;
};
}  // namespace uipc::core
