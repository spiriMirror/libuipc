namespace muda
{
template <typename T>
MUDA_INLINE void ComputeGraphVar<BufferView<T>>::update(const RWView& view)
{
    ComputeGraphVarBase::update();
    m_value = view;
}
template <typename T>
MUDA_INLINE ComputeGraphVar<BufferView<T>>& ComputeGraphVar<BufferView<T>>::operator=(const RWView& view)
{
    update(view);
    return *this;
}
}  // namespace muda