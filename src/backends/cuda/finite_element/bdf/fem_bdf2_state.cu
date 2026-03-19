#include <finite_element/bdf/fem_bdf2_state.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMBDF2State);

void FEMBDF2State::do_build()
{
    // just require the finite element method
    require<FiniteElementMethod>();
}

void FEMBDF2State::resize(SizeT size)
{
    m_impl.x_n_1s.resize(size);
    m_impl.v_n_1s.resize(size);
}

bool FEMBDF2State::Impl::dump(DumpInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_x_n_1s.dump(fmt::format("{}q_n_1.{}", path, frame), x_n_1s)  //
           && dump_v_n_1s.dump(fmt::format("{}qv_n_1.{}", path, frame), v_n_1s);
}

bool FEMBDF2State::Impl::try_recover(RecoverInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_x_n_1s.load(fmt::format("{}q_n_1.{}", path, frame))  //
           && dump_v_n_1s.load(fmt::format("{}qv_n_1.{}", path, frame));
}

void FEMBDF2State::Impl::apply_recover(RecoverInfo& info)
{
    dump_x_n_1s.apply_to(x_n_1s);
    dump_v_n_1s.apply_to(v_n_1s);
}

void FEMBDF2State::Impl::clear_recover(RecoverInfo& info)
{
    dump_x_n_1s.clean_up();
    dump_v_n_1s.clean_up();
}

bool FEMBDF2State::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool FEMBDF2State::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void FEMBDF2State::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void FEMBDF2State::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}
}  // namespace uipc::backend::cuda
