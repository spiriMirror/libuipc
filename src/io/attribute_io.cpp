#include <uipc/io/attribute_io.h>
#include <filesystem>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <fstream>
#include <uipc/common/list.h>
#include <sstream>
#include <igl/read_triangle_mesh.h>

namespace uipc::geometry
{
class AttributeIO::Interface
{
  public:
    virtual ~Interface()                                           = default;
    virtual void read(std::string_view name, IAttributeSlot& slot) = 0;
};

class ObjAttributeIO final : public AttributeIO::Interface
{
    template <typename T>
    using RowMajorMatrix =
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  public:
    ObjAttributeIO(std::string_view file)
        : m_file(file)
    {
        RowMajorMatrix<Float>  X;
        RowMajorMatrix<IndexT> F;
        if(!igl::read_triangle_mesh(string{file}, X, F))
        {
            throw AttributeIOError{fmt::format("Failed to load .obj file: {}", file)};
        }
        Vs.resize(X.rows());
        for(auto&& [i, v] : enumerate(Vs))
            v = X.row(i);
        Fs.resize(F.rows());
        for(auto&& [i, f] : enumerate(Fs))
            f = F.row(i);
    }

    virtual void read(std::string_view name, IAttributeSlot& slot) override
    {
        if(name == builtin::position)
        {
            auto& v3_slot = static_cast<AttributeSlot<Vector3>&>(slot);
            if(slot.size() != Vs.size())
            {
                throw AttributeIOError(fmt::format("Size mismatch: slot size {} vs. Vs size {}",
                                                   slot.size(),
                                                   Vs.size()));
            }
            auto v3_view = view(v3_slot);
            std::ranges::copy(span{Vs}, v3_view.begin());
        }
        else
        {
            throw AttributeIOError(fmt::format("Unsupported attribute name {}", name));
        }
    }

  private:
    std::string      m_file;
    vector<Vector3>  Vs;
    vector<Vector3i> Fs;
};

template <typename T>
U<AttributeIO::Interface> create_impl(std::string_view file)
{
    return uipc::static_pointer_cast<AttributeIO::Interface>(uipc::make_unique<T>(file));
}

AttributeIO::AttributeIO(std::string_view file)
{
    namespace fs = std::filesystem;
    auto path    = fs::path{file};
    if(!fs::exists(path))
    {
        throw AttributeIOError(fmt::format("File {} does not exist", file));
    }
    auto ext = path.extension();

    if(ext == ".obj")
    {
        m_impl = create_impl<ObjAttributeIO>(file);
    }
    else
    {
        throw AttributeIOError(
            fmt::format("Unsupported file format {} of file {}", ext.string(), file));
    }
}

AttributeIO::~AttributeIO() noexcept {}

void AttributeIO::read(std::string_view name, IAttributeSlot& slot)
{
    m_impl->read(name, slot);
}
}  // namespace uipc::geometry
