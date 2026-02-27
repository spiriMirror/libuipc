namespace muda
{
template <typename T>
MUDA_INLINE void ComputeGraphVar<Buffer3DView<T>>::update(const RWView& view)
{
    ComputeGraphVarBase::update();
    m_value = view;
}
template <typename T>
MUDA_INLINE ComputeGraphVar<Buffer3DView<T>>& ComputeGraphVar<Buffer3DView<T>>::operator=(const RWView& view)
{
    update(view);
    return *this;
}
}  // namespace muda