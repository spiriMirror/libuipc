/**
* @file matrix_market.h
* @brief Export block-sparse triplet matrices and dense vectors to Matrix Market format
* 
* Mtx format reference: https://math.nist.gov/MatrixMarket/formats.html
*/

#pragma once
#include <type_define.h>
#include <vector>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <map>
#include <fmt/printf.h>
#include <muda/buffer/buffer_view.h>
#include <muda/ext/linear_system/triplet_matrix_view.h>
#include <muda/ext/linear_system/dense_vector_view.h>

namespace uipc::backend::cuda
{
/**
 * @brief Export a block-sparse triplet matrix to Matrix Market coordinate format
 * 
 * Converts from block-sparse format (i, j, matrixBlockNxN) to Matrix Market
 * coordinate format (i, j, v) where each scalar value in the block is written separately.
 * All entries are exported, including zeros, to preserve the complete BCOO matrix structure.
 * 
 * @tparam T Value type (typically Float/double)
 * @tparam BlockDim Block dimension (e.g., 3 for 3x3 blocks)
 * @param filename Output filename
 * @param matrix Triplet matrix to export
 * @param one_based If true, use 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T, int BlockDim>
bool export_matrix_market(std::string_view                             filename,
                          const muda::CTripletMatrixView<T, BlockDim>& matrix,
                          bool one_based = true)
{
    using BlockMatrix = Eigen::Matrix<T, BlockDim, BlockDim>;

    // Copy data from device to host
    std::vector<int>         row_indices_host(matrix.triplet_count());
    std::vector<int>         col_indices_host(matrix.triplet_count());
    std::vector<BlockMatrix> values_host(matrix.triplet_count());

    matrix.row_indices().copy_to(row_indices_host.data());
    matrix.col_indices().copy_to(col_indices_host.data());
    matrix.values().copy_to(values_host.data());

    // Calculate total matrix dimensions (scalar, not block)
    int total_rows = matrix.total_rows() * BlockDim;
    int total_cols = matrix.total_cols() * BlockDim;
    int exact_nnz  = 0;
    // create fmt bufferauto
    auto out = fmt::memory_buffer();

    {
        // Count total non-zeros (each block contributes BlockDim * BlockDim entries)
        int total_nnz = matrix.triplet_count() * BlockDim * BlockDim;


        // Write coordinate entries
        int index_offset = one_based ? 1 : 0;


        for(size_t t = 0; t < matrix.triplet_count(); ++t)
        {
            int                block_i = row_indices_host[t];
            int                block_j = col_indices_host[t];
            const BlockMatrix& block   = values_host[t];

            // Convert block indices to scalar indices
            int scalar_i_base = block_i * BlockDim;
            int scalar_j_base = block_j * BlockDim;

            // Write each scalar entry in the block (including zeros)
            for(int bi = 0; bi < BlockDim; ++bi)
            {
                for(int bj = 0; bj < BlockDim; ++bj)
                {
                    T   value    = block(bi, bj);
                    int scalar_i = scalar_i_base + bi + index_offset;
                    int scalar_j = scalar_j_base + bj + index_offset;
                    // Write all entries, including zeros, to verify BCOO matrix structure
                    ++exact_nnz;
                    // full precision, scientific notation
                    fmt::format_to(std::back_inserter(out), "{} {} {:.17g}\n", scalar_i, scalar_j, value);
                }
            }
        }
    }

    // Open output file
    FILE* fp = std::fopen(std::string(filename).c_str(), "w");
    if(!fp)
    {
        return false;
    }

    // Write Matrix Market header using fmt::printf
    fmt::fprintf(fp, "%%MatrixMarket matrix coordinate real general\n");
    fmt::fprintf(fp, "%% Exported from block-sparse triplet matrix\n");
    fmt::fprintf(fp, "%% Block dimension: %dx%d\n", BlockDim, BlockDim);
    fmt::fprintf(fp, "%% Total rows: %d, Total cols: %d\n", total_rows, total_cols);
    fmt::fprintf(fp, "%% Total entries (including zeros): %d\n", exact_nnz);

    // Write dimensions and non-zero count
    fmt::fprintf(fp, "%d %d %d\n", total_rows, total_cols, exact_nnz);

    // Write the buffered entries to file
    std::fwrite(out.data(), 1, out.size(), fp);

    std::fclose(fp);
    return true;
}

/**
 * @brief Export a dense vector to Matrix Market array format
 * 
 * @param filename Output filename
 * @param vector Dense vector view to export
 * @param one_based If true, use 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T>
bool export_vector_market(std::string_view                 filename,
                          const muda::CDenseVectorView<T>& vector,
                          bool                             one_based = true)
{
    // Copy data from device to host
    std::vector<T> values_host(vector.size());
    vector.buffer_view().copy_to(values_host.data());

    // Open output file
    FILE* fp = std::fopen(std::string(filename).c_str(), "w");
    if(!fp)
    {
        return false;
    }

    // Write Matrix Market header using fmt::printf
    fmt::fprintf(fp, "%%MatrixMarket matrix array real general\n");
    fmt::fprintf(fp, "%% Exported from dense vector\n");
    fmt::fprintf(fp, "%% Vector size: %zu\n", vector.size());

    // Write dimensions (rows, 1 column)
    fmt::fprintf(fp, "%zu 1\n", vector.size());

    // Write values
    for(size_t i = 0; i < vector.size(); ++i)
    {
        fmt::fprintf(fp, "%.17g\n", values_host[i]);
    }

    std::fclose(fp);
    return true;
}

/**
 * @brief Export a buffer view to Matrix Market array format
 * 
 * @param filename Output filename
 * @param vector Buffer view to export
 * @param one_based If true, use 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T>
bool export_vector_market(std::string_view            filename,
                          const muda::CBufferView<T>& vector,
                          bool                        one_based = true)
{
    // Copy data from device to host
    std::vector<T> values_host(vector.size());
    vector.copy_to(values_host.data());

    // Open output file
    FILE* fp = std::fopen(std::string(filename).c_str(), "w");
    if(!fp)
    {
        return false;
    }

    // Write Matrix Market header using fmt::printf
    fmt::fprintf(fp, "%%MatrixMarket matrix array real general\n");
    fmt::fprintf(fp, "%% Exported from buffer view\n");
    fmt::fprintf(fp, "%% Vector size: %zu\n", vector.size());

    // Write dimensions (rows, 1 column)
    fmt::fprintf(fp, "%zu 1\n", vector.size());

    // Write values
    for(size_t i = 0; i < vector.size(); ++i)
    {
        fmt::fprintf(fp, "%.17g\n", values_host[i]);
    }

    std::fclose(fp);
    return true;
}

/**
 * @brief Export a span to Matrix Market array format
 * 
 * @param vector Span to export
 * @param filename Output filename
 * @param one_based If true, use 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T>
bool export_vector_market(span<const T> vector, std::string_view filename, bool one_based = true)
{
    // Open output file
    FILE* fp = std::fopen(std::string(filename).c_str(), "w");
    if(!fp)
    {
        return false;
    }
    // Write Matrix Market header using fmt::printf
    fmt::fprintf(fp, "%%MatrixMarket matrix array real general\n");
    fmt::fprintf(fp, "%% Exported from span\n");
    fmt::fprintf(fp, "%% Vector size: %zu\n", vector.size());
    // Write dimensions (rows, 1 column)
    fmt::fprintf(fp, "%zu 1\n", vector.size());
    // Write values
    for(size_t i = 0; i < vector.size(); ++i)
    {
        fmt::fprintf(fp, "%.17g\n", vector[i]);
    }
    std::fclose(fp);
    return true;
}

/**
 * @brief Import a sparse matrix from Matrix Market coordinate format
 * 
 * Converts from Matrix Market coordinate format (i, j, v) to block-sparse format
 * (i, j, matrixBlockNxN) by grouping scalar entries into blocks.
 * 
 * @tparam T Value type (typically Float/double)
 * @tparam BlockDim Block dimension (e.g., 3 for 3x3 blocks)
 * @param matrix Device triplet matrix to populate
 * @param filename Input filename
 * @param one_based If true, input uses 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T, int BlockDim>
bool import_matrix_market(muda::DeviceTripletMatrix<T, BlockDim>& matrix,
                          std::string_view                        filename,
                          bool one_based = true)
{
    using BlockMatrix = Eigen::Matrix<T, BlockDim, BlockDim>;

    std::ifstream file{std::string{filename}};
    if(!file.is_open())
    {
        return false;
    }

    std::string line;
    bool        header_read = false;
    int         rows = 0, cols = 0, nnz = 0;
    int         index_offset = one_based ? -1 : 0;  // Convert to 0-based

    // Read header and dimensions
    while(std::getline(file, line))
    {
        // Skip comments
        if(line.empty() || line[0] == '%')
            continue;

        if(!header_read)
        {
            std::istringstream iss(line);
            if(!(iss >> rows >> cols >> nnz))
            {
                file.close();
                return false;
            }
            header_read = true;
            break;
        }
    }

    if(!header_read || rows <= 0 || cols <= 0 || nnz <= 0)
    {
        file.close();
        return false;
    }

    // Read coordinate entries
    std::vector<std::tuple<int, int, T>> entries;
    entries.reserve(nnz);

    while(std::getline(file, line))
    {
        if(line.empty() || line[0] == '%')
            continue;

        std::istringstream iss(line);
        int                i, j;
        T                  value;
        if(iss >> i >> j >> value)
        {
            i += index_offset;  // Convert to 0-based
            j += index_offset;
            if(i >= 0 && i < rows && j >= 0 && j < cols)
            {
                entries.emplace_back(i, j, value);
            }
        }
    }
    file.close();

    // Group entries into blocks
    int block_rows = (rows + BlockDim - 1) / BlockDim;
    int block_cols = (cols + BlockDim - 1) / BlockDim;

    // Map from (block_i, block_j) to block matrix
    auto less = [](const std::pair<int, int>& a, const std::pair<int, int>& b)
    {
        if(a.first != b.first)
            return a.first < b.first;
        return a.second < b.second;
    };
    std::map<std::pair<int, int>, BlockMatrix, decltype(less)> block_map(less);

    for(const auto& [i, j, value] : entries)
    {
        int block_i = i / BlockDim;
        int block_j = j / BlockDim;
        int bi      = i % BlockDim;
        int bj      = j % BlockDim;

        auto key = std::make_pair(block_i, block_j);
        if(block_map.find(key) == block_map.end())
        {
            block_map[key] = BlockMatrix::Zero();
        }
        block_map[key](bi, bj) = value;
    }

    // Convert to triplet format
    matrix.reshape(block_rows, block_cols);
    matrix.resize_triplets(block_map.size());

    std::vector<int>         row_indices_host(block_map.size());
    std::vector<int>         col_indices_host(block_map.size());
    std::vector<BlockMatrix> values_host(block_map.size());

    size_t idx = 0;
    for(const auto& [key, block] : block_map)
    {
        row_indices_host[idx] = key.first;
        col_indices_host[idx] = key.second;
        values_host[idx]      = block;
        ++idx;
    }

    // Copy to device
    matrix.row_indices().copy_from(row_indices_host.data());
    matrix.col_indices().copy_from(col_indices_host.data());
    matrix.values().copy_from(values_host.data());

    return true;
}

/**
 * @brief Import a dense vector from Matrix Market array format
 * 
 * @param vector Device dense vector to populate
 * @param filename Input filename
 * @param one_based If true, input uses 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T>
bool import_vector_market(muda::DeviceDenseVector<T>& vector,
                          std::string_view            filename,
                          bool                        one_based = true)
{
    std::ifstream file{std::string{filename}};
    if(!file.is_open())
    {
        return false;
    }

    std::string line;
    bool        header_read = false;
    int         rows = 0, cols = 0;

    // Read header and dimensions
    while(std::getline(file, line))
    {
        // Skip comments
        if(line.empty() || line[0] == '%')
            continue;

        if(!header_read)
        {
            std::istringstream iss(line);
            if(!(iss >> rows >> cols))
            {
                file.close();
                return false;
            }
            if(cols != 1)
            {
                file.close();
                return false;
            }
            header_read = true;
            break;
        }
    }

    if(!header_read || rows <= 0)
    {
        file.close();
        return false;
    }

    // Read values
    std::vector<T> values_host;
    values_host.reserve(rows);

    while(std::getline(file, line))
    {
        if(line.empty() || line[0] == '%')
            continue;

        std::istringstream iss(line);
        T                  value;
        if(iss >> value)
        {
            values_host.push_back(value);
        }
    }
    file.close();

    if(values_host.size() != static_cast<size_t>(rows))
    {
        return false;
    }

    // Resize and copy to device
    vector.resize(rows);
    vector.buffer_view().copy_from(values_host.data());

    return true;
}

/**
 * @brief Import a dense vector from Matrix Market array format
 * 
 * @param vector Device buffer to populate
 * @param filename Input filename
 * @param one_based If true, input uses 1-based indexing (Matrix Market standard), else 0-based
 * @return true if successful, false otherwise
 */
template <typename T>
bool import_vector_market(muda::DeviceBuffer<T>& vector,
                          std::string_view       filename,
                          bool                   one_based = true)
{
    std::ifstream file{std::string{filename}};
    if(!file.is_open())
    {
        return false;
    }

    std::string line;
    bool        header_read = false;
    int         rows = 0, cols = 0;

    // Read header and dimensions
    while(std::getline(file, line))
    {
        // Skip comments
        if(line.empty() || line[0] == '%')
            continue;

        if(!header_read)
        {
            std::istringstream iss(line);
            if(!(iss >> rows >> cols))
            {
                file.close();
                return false;
            }
            if(cols != 1)
            {
                file.close();
                return false;
            }
            header_read = true;
            break;
        }
    }

    if(!header_read || rows <= 0)
    {
        file.close();
        return false;
    }

    // Read values
    std::vector<T> values_host;
    values_host.reserve(rows);

    while(std::getline(file, line))
    {
        if(line.empty() || line[0] == '%')
            continue;

        std::istringstream iss(line);
        T                  value;
        if(iss >> value)
        {
            values_host.push_back(value);
        }
    }
    file.close();

    if(values_host.size() != static_cast<size_t>(rows))
    {
        return false;
    }

    // Resize and copy to device
    vector.resize(rows);
    vector.view().copy_from(values_host.data());

    return true;
}
}  // namespace uipc::backend::cuda
