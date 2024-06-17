#pragma once
#include <uipc/geometry/simplicial_complex.h>
namespace uipc::geometry
{
class UIPC_CORE_API SpreadSheetIO
{
  public:
    SpreadSheetIO(std::string_view output_folder = "./");
    void write_json(std::string_view geo_name, const SimplicialComplex& simplicial_complex) const;
    void write_json(const SimplicialComplex& simplicial_complex) const;
    void write_csv(std::string_view geo_name, const SimplicialComplex& simplicial_complex) const;
    void write_csv(const SimplicialComplex& simplicial_complex) const;

  private:
    std::string m_output_folder;
};
}  // namespace uipc::geometry
