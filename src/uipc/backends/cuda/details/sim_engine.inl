namespace uipc::backend::cuda
{
template <std::derived_from<ISimSystem> T>
T* SimEngine::find()
{
    auto ptr = m_system_collection.find<T>();
    if(ptr)
        ptr->make_engine_aware();
    return ptr;
}
}  // namespace uipc::backend::cuda
