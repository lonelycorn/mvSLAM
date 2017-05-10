#include "unit-test.hpp"

#include <base/parameter-manager.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

using namespace unit_test;

UNIT_TEST(parameter_manager_io)
{
    const std::string filename("sample.param");
    const int variable_count = 6;
    const std::string DEFAULT_VALUE("VARIABLE_NOT_FOUND");

    // generate parameters
    std::unordered_map<std::string, std::string> module1, module2;

    module1["variable_int"] = "123";
    module1["variable_string"] = "abc";
    module1["variable_float"] = "123.45";

    module2["blah"] = "";
    module2["xxx"] = "//>/";
    module2["erzloi s"] = "123sadljlkj1   alsfdj";

    mvSLAM::ParameterManager::DEBUG_set_module_parameters(
            "module1", module1);

    mvSLAM::ParameterManager::DEBUG_set_module_parameters(
            "module2", module2);

    // clear file
    {
        std::fstream fout(filename, std::ios_base::out);
    }

    // save to file
    {
        int result = mvSLAM::ParameterManager::save_to_file(filename);
        ASSERT_TRUE(result == variable_count);
    }

    // clear ParameterManager
    mvSLAM::ParameterManager::DEBUG_set_module_parameters(
            "module1", std::unordered_map<std::string, std::string>());

    mvSLAM::ParameterManager::DEBUG_set_module_parameters(
            "module2", std::unordered_map<std::string, std::string>());

    // make sure it's cleared
    for (const auto &p : module1)
    {
        auto value = mvSLAM::ParameterManager::get_value(
                "module1", p.first, DEFAULT_VALUE);
        ASSERT_TRUE(value == DEFAULT_VALUE);
    }
    for (const auto &p : module2)
    {
        auto value = mvSLAM::ParameterManager::get_value(
                "module2", p.first, DEFAULT_VALUE);
        ASSERT_TRUE(value == DEFAULT_VALUE);
    }
    
    // load from file
    {
        int result = mvSLAM::ParameterManager::load_from_file(filename);
        ASSERT_TRUE(result == variable_count);
    }

    // check variables
    for (const auto &p : module1)
    {
        auto value = mvSLAM::ParameterManager::get_value(
                "module1", p.first, DEFAULT_VALUE);
        ASSERT_TRUE(value != DEFAULT_VALUE);
        ASSERT_TRUE(value == p.second);
    }

    for (const auto &p : module2)
    {
        auto value = mvSLAM::ParameterManager::get_value(
                "module2", p.first, DEFAULT_VALUE);
        ASSERT_TRUE(value != DEFAULT_VALUE);
        ASSERT_TRUE(value == p.second);
    }
    
    PASS();
}


int main()
{
    RUN_ALL_TESTS();
}
