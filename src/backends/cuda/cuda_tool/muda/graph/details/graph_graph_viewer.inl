namespace muda
{
MUDA_INLINE void ComputeGraphVar<GraphViewer>::update(const RWView& view)
{
    ComputeGraphVarBase::update();
    m_value = view;
}

MUDA_INLINE ComputeGraphVar<GraphViewer>& ComputeGraphVar<GraphViewer>::operator=(const RWView& view)
{
    update(view);
    return *this;
}

MUDA_INLINE void ComputeGraphVar<GraphViewer>::graphviz_def(
    std::ostream& o, const ComputeGraphGraphvizOptions& options) const
{
    graphviz_id(o, options);
    o << "[";
    if(!name().empty())
        o << "label=\"" << name() << "\",";
    o << options.graph_viewer_style;
    o << "]";
}
}  // namespace muda