#pragma once
namespace muda
{
namespace details
{
    MUDA_INLINE MUDA_GENERIC void cse_check_data(void* m_data, const char* m_name, const char* m_kernel_name) MUDA_NOEXCEPT
    {
        MUDA_KERNEL_ASSERT(m_data != nullptr, "cse[%s:%s]: data is null", m_name, m_kernel_name);
    }

    MUDA_INLINE MUDA_GENERIC void cse_check_dimi(int i, int m_dim_i, const char* m_name, const char* m_kernel_name) MUDA_NOEXCEPT
    {
        if(i < 0 || i >= m_dim_i)
            MUDA_KERNEL_ERROR("cse[%s:%s]: out of range, i=(%d), dim_i=(%d)", m_name, m_kernel_name, i, m_dim_i);
    }

    MUDA_INLINE MUDA_GENERIC void cse_check_dimj(int i, int j, int dimj, int m_dim_i, const char* m_name, const char* m_kernel_name) MUDA_NOEXCEPT
    {

        if(dimj < 0 || j < 0 || j >= dimj)
            MUDA_KERNEL_ERROR(
                "cse[%s:%s]: out of range, ij=(%d,%d), dim=(%d,%d)", m_name, m_kernel_name, i, j, m_dim_i, dimj);
    }

    MUDA_INLINE MUDA_GENERIC void cse_check_global_offset(
        int i, int j, int dimj, int global_offset, int m_dim_i, int m_ndata, const char* m_name, const char* m_kernel_name) MUDA_NOEXCEPT
    {

        if(global_offset < 0 || global_offset >= m_ndata)
            MUDA_KERNEL_ERROR("cse[%s:%s]: global_offset out of range, ij=(%d,%d), dim=(%d,%d), offset=(%d), ndata=(%d)",
                              m_name, m_kernel_name,
                              i,
                              j,
                              m_dim_i,
                              dimj,
                              global_offset,
                              m_ndata);
    }
}  // namespace details
}  // namespace muda