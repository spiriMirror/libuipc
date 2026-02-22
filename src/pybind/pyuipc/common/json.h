/***************************************************************************
* Copyright (c) 2019, Martin Renou                                         *
*                                                                          *
* Distributed under the terms of the BSD 3-Clause License.                 *
*                                                                          *
* The full license is in the file LICENSE, distributed with this software. *
****************************************************************************/

#ifndef NANOBIND_JSON_HPP
#define NANOBIND_JSON_HPP

#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace py = nanobind;
namespace nl = nlohmann;

namespace pyjson
{
inline py::object from_json(const nl::json& j)
{
    if(j.is_null())
    {
        return py::none();
    }
    else if(j.is_boolean())
    {
        return py::bool_(j.get<bool>());
    }
    else if(j.is_number_unsigned())
    {
        return py::int_(j.get<nl::json::number_unsigned_t>());
    }
    else if(j.is_number_integer())
    {
        return py::int_(j.get<nl::json::number_integer_t>());
    }
    else if(j.is_number_float())
    {
        return py::float_(j.get<double>());
    }
    else if(j.is_string())
    {
        return py::str(j.get<std::string>().c_str());
    }
    else if(j.is_array())
    {
        py::list obj;
        for(std::size_t i = 0; i < j.size(); i++)
        {
            obj.append(from_json(j[i]));
        }
        return obj;
    }
    else  // Object
    {
        py::dict obj;
        for(nl::json::const_iterator it = j.cbegin(); it != j.cend(); ++it)
        {
            obj[it.key().c_str()] = from_json(it.value());
        }
        return obj;
    }
}

inline nl::json to_json(const py::handle& obj)
{
    if(obj.ptr() == nullptr || obj.is_none())
    {
        return nullptr;
    }
    if(py::isinstance<py::bool_>(obj))
    {
        return py::cast<bool>(obj);
    }
    if(py::isinstance<py::int_>(obj))
    {
        try
        {
            nl::json::number_integer_t s = py::cast<nl::json::number_integer_t>(obj);
            return s;
        }
        catch(...)
        {
        }
        try
        {
            nl::json::number_unsigned_t u = py::cast<nl::json::number_unsigned_t>(obj);
            return u;
        }
        catch(...)
        {
        }
        throw std::runtime_error("to_json received an integer out of range for both nl::json::number_integer_t and nl::json::number_unsigned_t type");
    }
    if(py::isinstance<py::float_>(obj))
    {
        return py::cast<double>(obj);
    }
    if(py::isinstance<py::bytes>(obj))
    {
        py::module_ base64 = py::module_::import_("base64");
        return py::cast<std::string>(base64.attr("b64encode")(obj).attr("decode")("utf-8"));
    }
    if(py::isinstance<py::str>(obj))
    {
        return py::cast<std::string>(obj);
    }
    if(py::isinstance<py::tuple>(obj) || py::isinstance<py::list>(obj))
    {
        auto out = nl::json::array();
        for(const py::handle value : obj)
        {
            out.push_back(to_json(value));
        }
        return out;
    }
    if(py::isinstance<py::dict>(obj))
    {
        auto out = nl::json::object();
        for(const py::handle key : obj)
        {
            out[py::cast<std::string>(py::str(key))] = to_json(obj[key]);
        }
        return out;
    }
    throw std::runtime_error("to_json not implemented for this type of object");
}
}  // namespace pyjson

// nanobind caster
namespace nanobind::detail
{
    template <>
    struct type_caster<nl::json>
    {
      public:
        NB_TYPE_CASTER(nl::json, const_name("object"));

        bool from_python(handle src, uint8_t, cleanup_list*)
        {
            try
            {
                value = pyjson::to_json(src);
                return true;
            }
            catch(...)
            {
                return false;
            }
        }

        static handle from_cpp(nl::json src, rv_policy, cleanup_list*)
        {
            object obj = pyjson::from_json(src);
            return obj.release();
        }
    };
}  // namespace nanobind::detail

#endif
