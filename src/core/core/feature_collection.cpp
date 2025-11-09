#include <uipc/core/feature_collection.h>

namespace uipc::core
{
S<IFeature> FeatureCollection::find(std::string_view name) const
{
    auto it = m_features.find(std::string{name});
    if(it == m_features.end())
    {
        return nullptr;
    }
    else
    {
        it->second->on_required();
        return it->second;
    }
}

void FeatureCollection::insert(std::string_view name, S<IFeature> feature)
{
    m_features[std::string{name}] = feature;
}

bool FeatureCollection::contains(std::string_view name) const
{
    return m_features.find(std::string{name}) != m_features.end();
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
