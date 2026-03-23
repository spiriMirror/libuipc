#include <affine_body/bdf/abd_bdf2_state.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDBDF2State);

void ABDBDF2State::do_build()
{
    // just require the affine body dynamics
    require<AffineBodyDynamics>();
}

void ABDBDF2State::resize(SizeT size)
{
    m_impl.q_n_1s.resize(size);
    m_impl.q_v_n_1s.resize(size);
}

bool ABDBDF2State::Impl::dump(DumpInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_q_n_1s.dump(fmt::format("{}q_n_1.{}", path, frame), q_n_1s)  //
           && dump_q_v_n_1s.dump(fmt::format("{}qv_n_1.{}", path, frame), q_v_n_1s);
}

bool ABDBDF2State::Impl::try_recover(RecoverInfo& info)
{
    auto path  = info.dump_path(UIPC_RELATIVE_SOURCE_FILE);
    auto frame = info.frame();

    return dump_q_n_1s.load(fmt::format("{}q_n_1.{}", path, frame))  //
           && dump_q_v_n_1s.load(fmt::format("{}qv_n_1.{}", path, frame));
}

void ABDBDF2State::Impl::apply_recover(RecoverInfo& info)
{
    dump_q_n_1s.apply_to(q_n_1s);
    dump_q_v_n_1s.apply_to(q_v_n_1s);
}

void ABDBDF2State::Impl::clear_recover(RecoverInfo& info)
{
    dump_q_n_1s.clean_up();
    dump_q_v_n_1s.clean_up();
}

bool ABDBDF2State::do_dump(DumpInfo& info)
{
    return m_impl.dump(info);
}

bool ABDBDF2State::do_try_recover(RecoverInfo& info)
{
    return m_impl.try_recover(info);
}

void ABDBDF2State::do_apply_recover(RecoverInfo& info)
{
    m_impl.apply_recover(info);
}

void ABDBDF2State::do_clear_recover(RecoverInfo& info)
{
    m_impl.clear_recover(info);
}
}  // namespace uipc::backend::cuda
