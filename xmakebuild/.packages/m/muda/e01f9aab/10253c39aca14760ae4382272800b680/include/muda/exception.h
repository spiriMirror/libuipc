#pragma once
#include <muda/muda_def.h>
#include <exception>
#include <string>

namespace muda
{
class exception : public std::exception
{
    std::string m_what;

  public:
    exception(const std::string& msg)
        : m_what(msg)
    {
    }
    virtual char const* what() const
#ifdef MUDA_PLATFORM_LINUX
        MUDA_NOEXCEPT
#endif
    {
        return m_what.c_str();
    }
};

class not_implemented : public exception
{
  public:
    not_implemented(const std::string& msg)
        : exception(msg)
    {
    }
};

class invalid_argument : public exception
{
  public:
    invalid_argument(const std::string& msg)
        : exception(msg)
    {
    }
};

class out_of_range : public exception
{
  public:
    out_of_range(const std::string& msg)
        : exception(msg)
    {
    }
};

class runtime_error : public exception
{
  public:
    runtime_error(const std::string& msg)
        : exception(msg)
    {
    }
};

class logic_error : public exception
{
  public:
    logic_error(const std::string& msg)
        : exception(msg)
    {
    }
};
}  // namespace muda