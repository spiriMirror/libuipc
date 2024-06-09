namespace uipc::backend::cuda
{
template <std::derived_from<SimSystem> T>
T* SimEngine::find()
{
    return m_system_collection.find<T>();
}
}
