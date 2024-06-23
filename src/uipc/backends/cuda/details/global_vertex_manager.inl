namespace uipc::backend::cuda
{
template <typename T>
muda::BufferView<T> GlobalVertexManager::Impl::subview(muda::DeviceBuffer<T>& buffer,
                                                       SizeT index) const noexcept
{
    return buffer.view(register_vertex_offsets[index], register_vertex_counts[index]);
}
}  // namespace uipc::backend::cuda
