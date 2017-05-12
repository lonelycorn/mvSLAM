#include "string-manipulation.hpp"
#include <cctype>

namespace mvSLAM
{

static bool _trimmed_string_is_alphabet(const std::string &s)
{
    for (auto ch : s)
    {
        if (0 == std::isalpha(ch))
        {
            return false;
        }
    }
    return true;
}

std::string
string_trim_whitespaces(const std::string &s)
{
    if (s.size() == 0)
    {
        return s;
    }

    static const char *WHITESPACES = " \t\n\r\f\v";
    std::string t(s);

    // remove leading spaces
    t.erase(0, t.find_first_not_of(WHITESPACES));

    // remove trailing spaces
    t.erase(t.find_last_not_of(WHITESPACES) + 1);

    return t;
}

std::string
string_to_upper(const std::string &s)
{
    std::string t(s);
    for (auto &ch : t)
    {
        ch = std::toupper(ch);
    }
    return t;
}

std::string
string_to_lower(const std::string &s)
{
    std::string t(s);
    for (auto &ch : t)
    {
        ch = std::tolower(ch);
    }
    return t;
}

bool
string_is_alphabet(const std::string &s_)
{
    std::string s = string_trim_whitespaces(s_);
    return _trimmed_string_is_alphabet(s);
}

bool
string_is_scalar(const std::string &s_)
{
    std::string s = string_trim_whitespaces(s_);
    if (s.size() == 0)
    {
        return false;
    }

    int sign = 0;
    int digit = 0;
    int decimal_point = 0;
    size_t p = 0;

    while (p < s.size())
    {
        if (std::isdigit(s[p]))
        {
            ++digit;
        }
        else if (('+' == s[p]) || ('-' == s[p]))
        {
            ++sign;
            if (sign > 1)
            {
                return false;
            }
        }
        else if ('.' == s[p])
        {
            ++decimal_point;
            if (decimal_point > 1)
            {
                return false;
            }
        }
        else
        {
            return false;
        }
        ++p;
    }

    return (digit > 0);
}

bool
string_is_boolean(const std::string &s_)
{
    std::string s = string_trim_whitespaces(s_);
    if (!_trimmed_string_is_alphabet(s))
    {
        return false;
    }

    s = string_to_upper(s);
    return (("TRUE" == s) || ("FALSE" == s));
}

}
