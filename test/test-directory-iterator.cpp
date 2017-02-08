#include "unit-test.hpp"
#include <string>
#include <os/directory-iterator.hpp>
#include <iostream>

using namespace unit_test;

UNIT_TEST(sfm_two_images)
{
    const std::string directory("../data/sfm");
    const std::string extension("jpg");

    mvSLAM::DirectoryIterator it(directory, extension);
    
    {
        std::string fn = it.get_file_name();
        ASSERT_TRUE(fn == std::string("1.jpg"));
        bool has_next = it.next();
        ASSERT_TRUE(has_next);
    }

    {
        std::string fn = it.get_file_name();
        ASSERT_TRUE(fn == std::string("2.jpg"));
        bool has_next = it.next();
        ASSERT_TRUE(!has_next);
    }
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
