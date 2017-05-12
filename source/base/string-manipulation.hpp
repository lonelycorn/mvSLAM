#pragma once
#include <string>
namespace mvSLAM
{
/// remove leading and trailing whitespaces
std::string string_trim_whitespaces(const std::string &s);

/// convert all alphabets to uppercase.
std::string string_to_upper(const std::string &s);

/// convert all alphabets to lowercase.
std::string string_to_lower(const std::string &s);

// return true if the trimmed string contains only alphabets
bool string_is_alphabet(const std::string &s);

// TODO: make it work for scientific notation
/// return true if the trimmed string is a valid scalar (ordinary notation)
bool string_is_scalar(const std::string &s);

// return true if it the trimmed string is equavalent to "true" or "false"
bool string_is_boolean(const std::string &s);
}
