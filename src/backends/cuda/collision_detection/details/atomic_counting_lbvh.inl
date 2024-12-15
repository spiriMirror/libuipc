namespace uipc::backend::cuda
{
template <typename Pred>
void AtomicCountingLBVH::detect(Pred p, QueryBuffer& qbuffer)
{
    using namespace muda;

    if(m_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        ParallelFor(0, s)
            .file_line(__FILE__, __LINE__)
            .apply(m_aabbs.size(),
                   [lbvh   = m_lbvh.viewer().name("lbvh"),
                    aabbs  = m_aabbs.viewer().name("aabbs"),
                    cp_num = m_cp_num.viewer().name("cp_num"),
                    pairs  = qbuffer.m_pairs.viewer().name("pairs"),
                    p      = p] __device__(int i) mutable
                   {
                       auto N = aabbs.total_size();

                       auto aabb = aabbs(i);
                       lbvh.query(aabb,
                                  [&](uint32_t id)
                                  {
                                      if(id > i && p(i, id))
                                      {
                                          auto last = muda::atomic_add(cp_num.data(), 1);
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
    if(h_cp_num > qbuffer.m_pairs.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    qbuffer.m_size = h_cp_num;
}

template <typename Pred>
void AtomicCountingLBVH::query(muda::CBufferView<LinearBVHAABB> query_aabbs, Pred p, QueryBuffer& qbuffer)
{
    using namespace muda;

    if(m_aabbs.size() == 0 || query_aabbs.size() == 0)
    {
        qbuffer.m_size = 0;
        return;
    }

    auto do_query = [&]
    {
        cudaStream_t s = m_stream;
        BufferLaunch(s).fill(m_cp_num.view(), 0);

        ParallelFor(0, s)
            .file_line(__FILE__, __LINE__)
            .apply(query_aabbs.size(),
                   [lbvh   = m_lbvh.viewer().name("lbvh"),
                    aabbs  = query_aabbs.viewer().name("aabbs"),
                    cp_num = m_cp_num.viewer().name("cp_num"),
                    pairs  = qbuffer.m_pairs.viewer().name("pairs"),
                    p      = p] __device__(int i) mutable
                   {
                       auto N = aabbs.total_size();

                       auto aabb = aabbs(i);

                       lbvh.query(aabb,
                                  [&](uint32_t id)
                                  {
                                      if(p(i, id))
                                      {
                                          auto last = muda::atomic_add(cp_num.data(), 1);
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
    if(h_cp_num > qbuffer.size())
    {
        qbuffer.m_pairs.resize(h_cp_num * m_reserve_ratio);
        do_query();
    }

    qbuffer.m_size = h_cp_num;
}
}  // namespace uipc::backend::cuda
