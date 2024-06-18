namespace uipc::backend::cuda
{
template <std::derived_from<ISimSystem> T>
T* SimEngine::find()
{
    return m_system_collection.find<T>();
}
}  // namespace uipc::backend::cuda
