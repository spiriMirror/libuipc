namespace uipc::backend::cuda
{
namespace
{
    using namespace cuda_tool;

    template <typename Pred>
    __global__ void atomic_counting_lbvh_detect_kernel(LinearBVHViewer lbvh,
                                                       cuda_tool::CBufferView<LinearBVHAABB> aabbs,
                                                       cuda_tool::Dense<int> cp_num,
                                                       cuda_tool::BufferView<Vector2i> pairs,
                                                       Pred p,
                                                       int  n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto N = aabbs.total_size();

        auto aabb = aabbs(i);
        lbvh.query(aabb,
                   [&](uint32_t id)
                   {
                       if(id > i && p(i, id))
                       {
                           auto last = cuda_tool::atomic_add(cp_num.data(), 1);
                           if(last < pairs.total_size())
                               pairs(last) = Vector2i(i, id);
                       }
                   });
    }

    template <typename Pred>
    __global__ void atomic_counting_lbvh_query_kernel(LinearBVHViewer lbvh,
                                                      cuda_tool::CBufferView<LinearBVHAABB> aabbs,
                                                      cuda_tool::Dense<int> cp_num,
                                                      cuda_tool::BufferView<Vector2i> pairs,
                                                      Pred p,
                                                      int  n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto N = aabbs.total_size();

        auto aabb = aabbs(i);

        lbvh.query(aabb,
                   [&](uint32_t id)
                   {
                       if(p(i, id))
                       {
                           auto last = cuda_tool::atomic_add(cp_num.data(), 1);
                           if(last < pairs.total_size())
                               pairs(last) = Vector2i(i, id);
                       }
                   });
    }
}  // namespace

template <typename Pred>
void AtomicCountingLBVH::detect(Pred p, QueryBuffer& qbuffer)
{
    using namespace cuda_tool;

    if(m_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        int n = (int)m_aabbs.size();
        if(n > 0)
        {
            auto k = atomic_counting_lbvh_detect_kernel<Pred>;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, s>>>(
                m_lbvh.viewer(),
                m_aabbs.viewer(),
                m_cp_num.viewer(),
                qbuffer.m_pairs.viewer(),
                p,
                n);
        }
    };

    do_query();

    // get total number of pairs
    int h_cp_num = m_cp_num;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    qbuffer.m_size = h_cp_num;
}

template <typename Pred>
void AtomicCountingLBVH::query(cuda_tool::CBufferView<LinearBVHAABB> query_aabbs,
                               Pred         p,
                               QueryBuffer& qbuffer)
{
    using namespace cuda_tool;

    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        int n = (int)query_aabbs.size();
        if(n > 0)
        {
            auto k = atomic_counting_lbvh_query_kernel<Pred>;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, s>>>(
                m_lbvh.viewer(),
                query_aabbs.viewer(),
                m_cp_num.viewer(),
                qbuffer.m_pairs.viewer(),
                p,
                n);
        }
    };

    do_query();

    // get total number of pairs
    int h_cp_num = m_cp_num;
    // if failed, resize and retry
    if(h_cp_num > qbuffer.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda
