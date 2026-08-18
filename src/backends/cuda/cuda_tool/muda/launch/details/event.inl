namespace muda
{
MUDA_INLINE Event::Event(Flags<Bit> flag)
{
    checkCudaErrors(cudaEventCreateWithFlags(&m_handle, static_cast<unsigned int>(flag)));
}

MUDA_INLINE auto Event::query() const -> QueryResult
{
    auto res = cudaEventQuery(m_handle);
    if(res != cudaSuccess && res != cudaErrorNotReady)
        checkCudaErrors(res);
    return static_cast<QueryResult>(res);
}

MUDA_INLINE float muda::Event::elapsed_time(cudaEvent_t start, cudaEvent_t stop)
{
    float time;
    checkCudaErrors(cudaEventElapsedTime(&time, start, stop));
    return time;
}

MUDA_INLINE Event::~Event()
{
    if(m_handle)
        checkCudaErrors(cudaEventDestroy(m_handle));
}

MUDA_INLINE Event::Event(Event&& o) MUDA_NOEXCEPT : m_handle(o.m_handle)
{
    o.m_handle = nullptr;
}

MUDA_INLINE Event& Event::operator=(Event&& o) MUDA_NOEXCEPT
{
    if(this == &o)
        return *this;

    if(m_handle)
        checkCudaErrors(cudaEventDestroy(m_handle));

    m_handle   = o.m_handle;
    o.m_handle = nullptr;
    return *this;
}
}  // namespace muda