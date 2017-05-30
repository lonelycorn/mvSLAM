#include "unit-test.hpp"
#include <os/directory-iterator.hpp>
#include <iostream>
#include <string>
#include <unordered_set>

using namespace unit_test;

UNIT_TEST(directory_estimator)
{
    const std::string directory("../data/tsukuba");
    const std::string extension("jpg");
    const size_t total_frame_count = 5;

    mvSLAM::DirectoryIterator it(directory, extension);
    std::unordered_set<std::string> visited_filename;

    while (it.next())
    {
        std::string fn = it.get_file_name();
        visited_filename.insert(fn);
    }

    ASSERT_TRUE(visited_filename.size() == total_frame_count);
    for (size_t i = 0; i < total_frame_count; ++i)
    {
        std::string ground_truth_fn = std::to_string(i + 1) + "." + extension;
        ASSERT_TRUE(visited_filename.count(ground_truth_fn) == 1);
    }
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
