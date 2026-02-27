namespace muda
{
template <typename T>
MUDA_INLINE void ComputeGraphVar<VarView<T>>::update(const RWView& view)
{
    ComputeGraphVarBase::update();
    m_value = view;
}
template <typename T>
MUDA_INLINE ComputeGraphVar<VarView<T>>& ComputeGraphVar<VarView<T>>::operator=(const RWView& view)
{
    update(view);
    return *this;
}
}  // namespace muda