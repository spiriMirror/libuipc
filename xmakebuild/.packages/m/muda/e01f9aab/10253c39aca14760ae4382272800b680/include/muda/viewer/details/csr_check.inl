#pragma once
namespace muda
{
namespace details
{
    MUDA_INLINE MUDA_GENERIC void csr_check_range(
        int row, int col, int m_rows, int m_cols, const char* m_name, const char* m_kernel_name) MUDA_NOEXCEPT
    {
        if(row < 0 || row >= m_rows || col < 0 || col >= m_cols)
        {
            MUDA_KERNEL_ERROR("csr[%s:%s]: row/col index out of range: index=(%d,%d) dim_=(%d,%d)",
                              m_name,
                              m_kernel_name,
                              row,
                              col,
                              m_rows,
                              m_cols);
        }
    }

    MUDA_INLINE MUDA_GENERIC void csr_check_row(int         row,
                                                int         m_rows,
                                                const char* m_name,
                                                const char* m_kernel_name) MUDA_NOEXCEPT
    {
        if(row < 0 || row >= m_rows)
        {
            MUDA_KERNEL_ERROR("csr[%s:%s]: row index out of range: index=(%d) rows=(%d)",
                              m_name,
                              m_kernel_name,
                              row,
                              m_rows);
        }
    }

    MUDA_INLINE MUDA_GENERIC void csr_check_local_offset(int         row,
                                                         int         offset,
                                                         int         m_rows,
                                                         const int*  m_rowPtr,
                                                         const char* m_name,
                                                         const char* m_kernel_name) MUDA_NOEXCEPT
    {
        if(row < 0 || row >= m_rows || offset < 0
           || offset >= m_rowPtr[row + 1] - m_rowPtr[row])
        {
            MUDA_KERNEL_ERROR(
                "csr[%s:%s]: 'rowPtr[row] + offset > rowPtr[row+1]' out of range:"
                "row=%d, offset=%d, rowPtr[row]=%d, rowPtr[row+1]=%d",
                m_name,
                m_kernel_name,
                row,
                offset,
                m_rowPtr[row],
                m_rowPtr[row + 1]);
        }
    }

    MUDA_INLINE MUDA_GENERIC void csr_check_global_offset(int globalOffset,
                                                          int m_nnz,
                                                          const char* m_name,
                                                          const char* m_kernel_name) MUDA_NOEXCEPT
    {
        if(globalOffset < 0 || globalOffset >= m_nnz)
        {
            MUDA_KERNEL_ERROR("csr[%s]: globalOffset out of range: globalOffset=%d, nnz=%d",
                              m_name,
                              m_kernel_name,
                              globalOffset,
                              m_nnz);
        }
    }
}  // namespace details
}  // namespace muda