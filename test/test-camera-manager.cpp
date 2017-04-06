#include "unit-test.hpp"
#include "unit-test-helper.hpp"
#include <front-end/camera-manager.hpp>

#include <iostream>
#include <fstream>
#include <string>

using namespace unit_test;

UNIT_TEST(camera_manager_default_camera)
{
    const mvSLAM::ScalarType tolerance(1e-5);
    auto &c = mvSLAM::CameraManager::get_camera();
    auto &K = c.get_intrinsics();
    auto &P = c.get_extrinsics();

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            if (i == j)
            {
                ASSERT_EQUAL(K(i, j), 1, tolerance);
            }
            else
            {
                ASSERT_EQUAL(K(i, j), 0, tolerance);
            }
        }

    bool result = check_similar_SE3(P, mvSLAM::SE3(), tolerance);
    ASSERT_TRUE(result);

    PASS();
}

UNIT_TEST(camera_manager_io)
{
    const std::string input_filename("../data/camera_config/sample.txt");
    const std::string output_filename("sample.txt.out");

    mvSLAM::CameraManager::load_from_file(input_filename);

    mvSLAM::CameraManager::save_to_file(output_filename);

    // byte-by-byte comparison
    std::ifstream f1(input_filename);
    std::ifstream f2(output_filename);
    
    while (!f1.eof() && !f2.eof())
    {
        char c1, c2;
        f1.get(c1);
        f2.get(c2);
        if (c1 != c2)
            std::cout<<"c1 = '"<<c1<<"'\nc2 = '"<<c2<<"'"<<std::endl;
        ASSERT_TRUE(c1 == c2);
    }

    ASSERT_TRUE(f1.eof() && f2.eof());

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
