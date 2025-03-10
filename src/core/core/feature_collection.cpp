#include <uipc/core/feature_collection.h>

namespace uipc::core
{
S<IFeature> FeatureCollection::find(std::string_view name) const
{
    auto it = m_features.find(std::string{name});
    return it == m_features.end() ? nullptr : it->second;
}

void FeatureCollection::insert(std::string_view name, S<IFeature> feature)
{
    m_features[std::string{name}] = feature;
}

Json FeatureCollection::to_json() const
{
    Json j = Json::array();
    for(const auto& [name, feature] : m_features)
    {
        j.push_back(name);
    }
    return j;
}
}  // namespace uipc::core
