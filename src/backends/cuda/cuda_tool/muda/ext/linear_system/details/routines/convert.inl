namespace muda
{
// Triplet -> BCOO
template <typename T, int M, int N>
void LinearSystemContext::convert(const DeviceTripletMatrix<T, M, N>& from,
                                  DeviceBCOOMatrix<T, M, N>&          to)
{
    m_converter.convert(from, to);
}

// BCOO -> Dense Matrix
template <typename T, int M, int N>
void LinearSystemContext::convert(const DeviceBCOOMatrix<T, M, N>& from,
                                  DeviceDenseMatrix<T>&            to,
                                  bool clear_dense_matrix)
{
    m_converter.convert(from, to, clear_dense_matrix);
}

// BCOO -> COO
template <typename T, int M, int N>
void LinearSystemContext::convert(const DeviceBCOOMatrix<T, M, N>& from,
                                  DeviceCOOMatrix<T>&              to)
{
    m_converter.convert(from, to);
}

// BCOO -> BSR
template <typename T, int N>
void LinearSystemContext::convert(const DeviceBCOOMatrix<T, N, N>& from,
                                  DeviceBSRMatrix<T, N>&           to)
{
    m_converter.convert(from, to);
}

// Doublet -> BCOO
template <typename T, int N>
void LinearSystemContext::convert(const DeviceDoubletVector<T, N>& from,
                                  DeviceBCOOVector<T, N>&          to)
{
    m_converter.convert(from, to);
}

// BCOO -> Dense Vector
template <typename T, int N>
void LinearSystemContext::convert(const DeviceBCOOVector<T, N>& from,
                                  DeviceDenseVector<T>&         to,
                                  bool clear_dense_vector)
{
    m_converter.convert(from, to, clear_dense_vector);
}

// Doublet -> Dense Vector
template <typename T, int N>
void LinearSystemContext::convert(const DeviceDoubletVector<T, N>& from,
                                  DeviceDenseVector<T>&            to,
                                  bool clear_dense_vector)
{
    m_converter.convert(from, to, clear_dense_vector);
}

// BSR -> CSR
template <typename T, int N>
void LinearSystemContext::convert(const DeviceBSRMatrix<T, N>& from, DeviceCSRMatrix<T>& to)
{
    m_converter.convert(from, to);
}

// Triplet -> COO
template <typename T>
void LinearSystemContext::convert(const DeviceTripletMatrix<T, 1>& from,
                                  DeviceCOOMatrix<T>&              to)
{
    m_converter.convert(from, to);
}

// COO -> Dense Matrix
template <typename T>
void LinearSystemContext::convert(const DeviceCOOMatrix<T>& from,
                                  DeviceDenseMatrix<T>&     to,
                                  bool                      clear_dense_matrix)
{
    m_converter.convert(from, to, clear_dense_matrix);
}

// COO -> CSR
template <typename T>
void LinearSystemContext::convert(const DeviceCOOMatrix<T>& from, DeviceCSRMatrix<T>& to)
{
    m_converter.convert(from, to);
}
template <typename T>
void LinearSystemContext::convert(DeviceCOOMatrix<T>&& from, DeviceCSRMatrix<T>& to)
{
    m_converter.convert(std::move(from), to);
}

// Doublet -> COO
template <typename T>
void LinearSystemContext::convert(const DeviceDoubletVector<T, 1>& from,
                                  DeviceCOOVector<T>&              to)
{
    m_converter.convert(from, to);
}

// COO -> Dense Vector
template <typename T>
void LinearSystemContext::convert(const DeviceCOOVector<T>& from,
                                  DeviceDenseVector<T>&     to,
                                  bool                      clear_dense_vector)
{
    m_converter.convert(from, to, clear_dense_vector);
}
template <typename T>
void LinearSystemContext::convert(const DeviceDoubletVector<T, 1>& from,
                                  DeviceDenseVector<T>&            to,
                                  bool clear_dense_vector)
{
    m_converter.convert(from, to, clear_dense_vector);
}
}  // namespace muda