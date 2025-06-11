#include <uipc/common/format.h>

namespace nlohmann
{
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void adl_serializer<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>::to_json(
    json& j, const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
{
    j = json::array();
    for(int i = 0; i < m.rows(); ++i)
    {
        j.push_back(json::array());
        auto& json_row = j.back();

        for(int j = 0; j < m.cols(); ++j)
        {
            json_row.push_back(m(i, j));
        }
    }
}

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void adl_serializer<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>>::from_json(
    const json& j, Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
{
    // check the row/column size
    if constexpr(Rows != Eigen::Dynamic)
    {
        if(j.size() != Rows)
        {
            throw uipc::JsonIOError(
                fmt::format("Unexpected row size: {} != {}", j.size(), Rows));
        }
    }


    for(int i = 0; i < j.size(); ++i)
    {
        if constexpr(Rows == Eigen::Dynamic || Cols == Eigen::Dynamic)
        {
            if(i == 0)
            {
                m.resize(j.size(), j[i].size());
            }
        }

        auto& json_row = j[i];

        if(m.cols() != json_row.size())
        {
            throw uipc::JsonIOError(fmt::format(
                "Unexpected column size: {} != {}, in row {}.", json_row.size(), m.cols(), i));
        }

        for(int j = 0; j < json_row.size(); ++j)
        {
            json_row[j].get_to(m(i, j));
        }
    }
}
}  // namespace nlohmann
