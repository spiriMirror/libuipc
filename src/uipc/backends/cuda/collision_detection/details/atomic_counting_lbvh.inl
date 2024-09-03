namespace uipc::backend::cuda
{
template <typename GetQueryAABB, typename ObjectT, typename Pred>
muda::BufferView<Vector2i> AtomicCountingLBVH::query(muda::CBufferView<ObjectT> objs,
                                                     GetQueryAABB get_aabb,
                                                     Pred         p)
{
    using namespace muda;

    if(m_aabbs.size() == 0)
        return m_pairs.view(0, 0);

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;

        BufferLaunch(s).fill(m_cp_num.view(), 0);

        ParallelFor(0, s)
            .kernel_name(__FUNCTION__)
            .apply(objs.size(),
                   [m_lbvh   = m_lbvh.viewer().name("lbvh"),
                    objs     = objs.viewer().name("objs"),
                    cp_num   = m_cp_num.viewer().name("cp_num"),
                    pairs    = m_pairs.viewer().name("pairs"),
                    get_aabb = get_aabb,
                    p        = p] __device__(int i) mutable
                   {
                       AABB aabb = get_aabb(objs(i));

                       m_lbvh.query(aabb,
                                    [&](uint32_t id)
                                    {
                                        if(p(i, id))
                                        {
                                            auto last =
                                                muda::atomic_add(cp_num.data(), 1);
                                            if(last < pairs.total_size())
                                                pairs(last) = Vector2i(i, id);
                                        }
                                    });
                   });
    };

    do_query();

    // get total number of pairs
    int h_cp_num = m_cp_num;
    // if failed, resize and retry
    if(h_cp_num > m_pairs.size())
    {
        m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    return m_pairs.view(0, h_cp_num);
}

template <typename Pred>
muda::BufferView<Vector2i> AtomicCountingLBVH::detect(Pred p)
{
    using namespace muda;

    if(m_aabbs.size() == 0)
        return m_pairs.view(0, 0);

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        ParallelFor(0, s)
            .kernel_name(__FUNCTION__)
            .apply(m_aabbs.size(),
                   [m_lbvh   = m_lbvh.viewer().name("lbvh"),
                    aabbs    = m_aabbs.viewer().name("aabbs"),
                    m_cp_num = m_cp_num.viewer().name("cp_num"),
                    m_pairs  = m_pairs.viewer().name("pairs"),
                    p        = p] __device__(int i) mutable
                   {
                       auto N = aabbs.total_size();

                       auto aabb = aabbs(i);
                       m_lbvh.query(aabb,
                                    [&](uint32_t id)
                                    {
                                        if(id > i && p(i, id))
                                        {
                                            auto last =
                                                muda::atomic_add(m_cp_num.data(), 1);
                                            if(last < m_pairs.total_size())
                                                m_pairs(last) = Vector2i(i, id);
                                        }
                                    });
                   });
    };

    do_query();

    // get total number of pairs
    int h_cp_num = m_cp_num;
    // if failed, resize and retry
    if(h_cp_num > m_pairs.size())
    {
        m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    return m_pairs.view(0, h_cp_num);
}

template <typename Pred>
muda::BufferView<Vector2i> AtomicCountingLBVH::query(muda::CBufferView<LinearBVHAABB> query_aabbs,
                                                     Pred p)
{
    using namespace muda;

    if(m_aabbs.size() == 0)
        return m_pairs.view(0, 0);

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        ParallelFor(0, s)
            .kernel_name(__FUNCTION__)
            .apply(query_aabbs.size(),
                   [m_lbvh   = m_lbvh.viewer().name("lbvh"),
                    aabbs    = query_aabbs.viewer().name("aabbs"),
                    m_cp_num = m_cp_num.viewer().name("cp_num"),
                    m_pairs  = m_pairs.viewer().name("pairs"),
                    p        = p] __device__(int i) mutable
                   {
                       auto N = aabbs.total_size();

                       auto aabb = aabbs(i);

                       m_lbvh.query(aabb,
                                    [&](uint32_t id)
                                    {
                                        if(p(i, id))
                                        {
                                            auto last =
                                                muda::atomic_add(m_cp_num.data(), 1);
                                            if(last < m_pairs.total_size())
                                                m_pairs(last) = Vector2i(i, id);
                                        }
                                    });
                   });
    };

    do_query();

    // get total number of pairs
    int h_cp_num = m_cp_num;
    // if failed, resize and retry
    if(h_cp_num > m_pairs.size())
    {
        m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    return m_pairs.view(0, h_cp_num);
}
}  // namespace uipc::backend::cuda
