#pragma once
/**
 */

#include <iostream>
#include <string>
#include <cassert>
#include <vector>

namespace unit_test
{

/** function prototype of unit tests
 */
typedef bool (*unit_test_fcn_ptr)();

std::vector<unit_test_fcn_ptr> unit_test_fcns;
std::vector<std::string> unit_test_fcn_names;

/** MACRO to register a unit test.
 */
#define REGISTER_UNIT_TEST(unit_test) \
class __class_register_##unit_test\
{\
public: \
    __class_register_##unit_test() \
    {\
        /*std::cout<<"Registering unit test: "<<#unit_test<<std::endl; */\
        unit_test_fcns.push_back(unit_test); \
        unit_test_fcn_names.push_back(std::string(#unit_test)); \
    }\
} __instance_register_##unit_test;

/** Declare a unit test and register it.
 */
#define UNIT_TEST(unit_test_name) \
bool unit_test_name(); \
REGISTER_UNIT_TEST(unit_test_name); \
bool unit_test_name()

/** Run all registered unit test.
 * @return true if all tests passed; otherwise false
 */
bool run_all_tests()
{
    // sanity check
    assert(unit_test_fcns.size() == unit_test_fcn_names.size());

    std::size_t passed_count = 0;
    std::size_t test_count = unit_test_fcns.size();
    std::cout<<"\nStarting all unit tests..."<<std::endl;
    for (std::size_t i = 0; i < test_count; ++i)
    {
        try
        {
            bool result = (*unit_test_fcns[i])();
            std::cout<<"    ["<<unit_test_fcn_names[i]<<"] "
                     <<(result ? "PASSED" : "FAILED")<<std::endl;
            passed_count += (result ? 1 : 0);
        }
        catch (std::exception &e)
        {
            std::cout<<"    ["<<unit_test_fcn_names[i]<<"] exception raised:\n"
                     <<e.what()<<std::endl;
        }
        catch (...)
        {
            std::cout<<"    ["<<unit_test_fcn_names[i]<<"] unknown exception raised"
                     <<std::endl;
        }
    }
    std::cout<<"Done testing; passed "<<passed_count<<" / "<<test_count<<"\n"<<std::endl;
    return passed_count == test_count;
}

/** MACRO to run all registered unit tests and return
 */
#define RUN_ALL_TESTS() \
{ \
    bool all_passed = unit_test::run_all_tests(); \
    return (all_passed) ? 0 : 1; \
}

/*
#define EXPECT_TRUE(x) \
{ \
    if (!(x)) \
    { \
        std::cout<<"EXPECT TURE: "<<#x<<std::endl; \
    }\
}
*/

#define ASSERT_TRUE(condition) \
{ \
    if (!(condition)) \
    { \
        std::cout<<"ASSERT TURE: "<<#condition<<std::endl; \
        return false; \
    } \
}

#define ASSERT_EQUAL(lhs, rhs, tolerance) \
{\
    if (((lhs) < (rhs) - (tolerance)) || ((lhs) > (rhs) + (tolerance))) \
    { \
        std::cout<<"EQUAL_TO: "<<#lhs<<" != "<<#rhs<<" (tol = "<<tolerance<<")"<<std::endl; \
        return false; \
    } \
}

/// Fail the unit test with error message printed to stdout
#define FAIL(error_msg) \
{ \
    std::cout<<"FAIL: "<<error_msg<<std::endl; \
    return false; \
}

/// Pass the unit test.
#define PASS(not_used) \
{ \
    return true; \
}

} // namespace unit_test

