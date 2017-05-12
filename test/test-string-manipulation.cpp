#include "unit-test.hpp"

#include <base/string-manipulation.hpp>

#include <string>
#include <iostream>

using namespace unit_test;

UNIT_TEST(string_trim_whitespaces)
{
    const std::string trimmed_string("abc lkjalskfdj");
    const std::string s("  \t" + trimmed_string + "\r\n  \t");
    ASSERT_TRUE(trimmed_string == mvSLAM::string_trim_whitespaces(s));
    PASS();
}

UNIT_TEST(string_to_upper)
{
    const std::string s1("aAbB2 sf");
    const std::string s2("AABB2 SF");
    ASSERT_TRUE(s2 == mvSLAM::string_to_upper(s1));
    PASS();
}

UNIT_TEST(string_to_lower)
{
    const std::string s1("Xx 123a?c  ");
    const std::string s2("xx 123a?c  ");
    ASSERT_TRUE(s2 == mvSLAM::string_to_lower(s1));
    PASS();
}

UNIT_TEST(string_is_alphabet)
{
    {
        const std::string s("123  asdf");
        ASSERT_TRUE(!mvSLAM::string_is_alphabet(s))
    }
    {
        const std::string s("aabbcc");
        ASSERT_TRUE(mvSLAM::string_is_alphabet(s))
    }
    {
        const std::string s("\taabbcc ");
        ASSERT_TRUE(mvSLAM::string_is_alphabet(s))
    }
    {
        const std::string s(" aa~ bbcc ");
        ASSERT_TRUE(!mvSLAM::string_is_alphabet(s))
    }
    {
        const std::string s(" 12.33");
        ASSERT_TRUE(!mvSLAM::string_is_alphabet(s))
    }
    PASS();
}

UNIT_TEST(string_is_scalar)
{
    {
        const std::string s("+123");
        ASSERT_TRUE(mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s("-1.23");
        ASSERT_TRUE(mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s("-1.23 ");
        ASSERT_TRUE(mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s(" .04");
        ASSERT_TRUE(mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s(" .0.4");
        ASSERT_TRUE(!mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s("3.5e-2");
        // FIXME: this really is a scalar; it's just not using oridnary notation
        ASSERT_TRUE(!mvSLAM::string_is_scalar(s));
    }
    {
        const std::string s("aa c123");
        ASSERT_TRUE(!mvSLAM::string_is_scalar(s));
    }
    PASS();
}

UNIT_TEST(string_is_boolean)
{
    {
        const std::string s("aa c123");
        ASSERT_TRUE(!mvSLAM::string_is_boolean(s));
    }
    {
        const std::string s("  true ");
        ASSERT_TRUE(mvSLAM::string_is_boolean(s));
    }
    {
        const std::string s(" FaLse ");
        ASSERT_TRUE(mvSLAM::string_is_boolean(s));
    }
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
