namespace muda
{
enum class LinearSystemReorderMethod
{
    None       = 0,
    Symrcm     = 1,
    Symamd     = 2,
    Csrmetisnd = 3,
};
class LinearSystemSolveReorder
{
    LinearSystemReorderMethod m_reorder_method = LinearSystemReorderMethod::None;

  public:
    LinearSystemReorderMethod reorder_method() const
    {
        return m_reorder_method;
    }
    void reoder_method(LinearSystemReorderMethod method)
    {
        m_reorder_method = method;
    }
    int reorder_method_int() const
    {
        return static_cast<int>(m_reorder_method);
    }
};
}  // namespace muda