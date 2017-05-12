#pragma once

#include <system-config.hpp>
#include <base/string-manipulation.hpp>

#include <string>
#include <cstdint>

namespace mvSLAM
{

/*===== to int ======*/
template <typename T>
int convert_to_int(const T &v)
{
    return static_cast<int>(v);
}

/* NOTE: fully specialized template functions are no different than regular
 *       functions. The compiler would complain about 'multiple definition'
 *       if a regular function is defined in the header file and the header
 *       file is included in multiple cpp files. To comply with the One
 *       Definition Rule, we have to add 'inline' to the declaration of
 *       fully specialized template functions. This 'inline' does not mean
 *       the compiler would expand the function in-place, though.
 */
template <> inline
int convert_to_int(const std::string &v)
{
    return std::stoi(v);
}

/*===== to int ======*/
template <typename T>
ScalarType convert_to_ScalarType(const T &v)
{
    return static_cast<ScalarType>(v);
}

template <> inline
ScalarType convert_to_ScalarType(const std::string &v)
{
    double double_value = std::stod(v);
    return convert_to_ScalarType(double_value);
}

/*===== to bool ======*/
template <typename NumericType>
bool convert_to_bool(const NumericType &v)
{
    return (v > 0);
}

/// return true if @p v is a positive scalar, or equavalence of "true"
template <> inline
bool convert_to_bool(const std::string &v_)
{
    if (v_.size() == 0)
    {
        return false;
    }
    std::string v = string_trim_whitespaces(v_);
    if (string_is_scalar(v))
    {
        return (convert_to_bool(convert_to_ScalarType(v)));
    }
    if (string_is_boolean(v))
    {
        return ("TRUE" == string_to_upper(v));
    }
    return false;
}

/*===== Generic Conversion =====*/
template <typename ToType, typename FromType>
struct Conversion;

/// no-op
template <typename T>
struct Conversion<T, T>
{
    static inline T convert(const T &v)
    {
        return v;
    }
};

template <typename FromType>
struct Conversion<int, FromType>
{
    static inline int convert(const FromType &v)
    {
        return convert_to_int(v);
    }
};

template <typename FromType>
struct Conversion<ScalarType, FromType>
{
    static inline int convert(const FromType &v)
    {
        return convert_to_ScalarType(v);
    }
};

template <typename FromType>
struct Conversion<bool, FromType>
{
    static inline int convert(const FromType &v)
    {
        return convert_to_bool(v);
    }
};

}
