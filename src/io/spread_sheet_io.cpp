#include <uipc/io/spread_sheet_io.h>
#include <filesystem>
#include <fstream>
#include <uipc/common/enumerate.h>

#include <fmt/ranges.h>

namespace uipc::geometry
{
namespace fs = std::filesystem;

SpreadSheetIO::SpreadSheetIO(std::string_view output_folder)
    : m_output_folder(output_folder)
{
    fs::exists(m_output_folder) || fs::create_directories(m_output_folder);
}

void SpreadSheetIO::write_json(std::string_view geo_name, const Geometry& geo) const
{
    constexpr int dump_indent = 4;
    auto          json        = geo.to_json();
    auto file = fs::path(m_output_folder) / fmt::format("{}.json", geo_name);

    {
        std::ofstream out(file);
        out << json.dump(dump_indent);
    }

    auto abs_file = fs::absolute(file);
    spdlog::info("Write spreadsheet(.json) to {}", abs_file.string());
}

void SpreadSheetIO::write_json(const Geometry& geo) const
{
    write_json("spreadsheet", geo);
}

void SpreadSheetIO::write_csv(std::string_view geo_name, const Geometry& geo) const
{
    auto folder = fs::path(m_output_folder) / fmt::format("{}", geo_name);
    fs::exists(folder) || fs::create_directories(folder);

    auto json = geo.to_json();

    for(auto& [key, J] : json.items())
    {
        auto          file = folder / fmt::format("{}.csv", key);
        std::ofstream out(file);

        auto attr_size = J.size();

        // find 'topo' attribute

        IndexT topo_idx = -1;

        for(auto&& [i, attr] : enumerate(J))
        {
            attr["name"] == "topo" ? topo_idx = i : topo_idx;
        }

        vector<string> row(attr_size);

        auto swap_topo_and_write = [&out, &row, topo_idx]()
        {
            if(topo_idx > 0)  // swap topo to the first column
            {
                std::swap(row[0], row[topo_idx]);
            }

            out << fmt::format("{}\n", fmt::join(row, ","));
        };

        // write header
        for(auto&& [i, attr] : enumerate(J))
        {
            row[i] = attr["name"].get<string>();
        }

        swap_topo_and_write();

        // write data
        SizeT i_size = J.size();
        SizeT j_size = 0;
        for(auto&& [i, attr] : enumerate(J))
        {
            j_size = std::max(j_size, attr["values"].size());
        }

        for(SizeT j = 0; j < j_size; ++j)
        {
            for(auto&& [i, attr] : enumerate(J))
            {
                auto& values = attr["values"];
                auto  j_size = values.size();

                auto& cell = row[i];

                if(values.size() <= j)
                {
                    cell = "";
                    continue;
                }

                auto& val = values[j];
                if(val.is_array())
                {
                    if(key == "vertices")
                    {
                        int a = 0;
                    }
                    cell = fmt::format(R"("{}")", val.dump());
                }
                else
                {
                    cell = val.dump();
                }
            }

            swap_topo_and_write();
        }
    }

    auto abs_folder = fs::absolute(folder);
    spdlog::info("Write spreadsheets(.csv) to {}", abs_folder.string());
}

void SpreadSheetIO::write_csv(const Geometry& geo) const
{
    write_csv("spreadsheet", geo);
}
}  // namespace uipc::geometry
