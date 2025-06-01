#pragma once
#include <uipc/geometry/geometry.h>
namespace uipc::geometry
{
/**
 * @brief A class for writing geometry spreadsheet.
 * 
 * @code
 *  SimplicialComplexIO io;
 *  auto mesh = io.read("cube.msh");
 *  SpreadSheetIO sio;
 *  // dump to csv
 *  sio.write_csv("spreadsheet", mesh);
 *  // dump to json
 *  sio.write_json("spreadsheet", mesh);
 * @endcode
 */
class UIPC_IO_API SpreadSheetIO
{
  public:
    SpreadSheetIO(std::string_view output_folder = "./");
    void write_json(std::string_view geo_name, const Geometry& geo) const;
    void write_json(const Geometry& geo) const;
    void write_csv(std::string_view geo_name, const Geometry& geo) const;
    void write_csv(const Geometry& geo) const;

  private:
    string m_output_folder;
};
}  // namespace uipc::geometry
