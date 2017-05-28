#include "unit-test.hpp"
#include "unit-test-helper.hpp"
#include <string>
#include <front-end/frame-manager.hpp>
#include <front-end/image-pair.hpp>
#include <iostream>
#include <set>

using namespace unit_test;

UNIT_TEST(image_pair)
{
    const mvSLAM::ScalarType tolerance(1e-3);
    const std::string directory("../data/tsukuba/");
    const std::string extension(".jpg");
    const std::string pair_frame_filename(directory + std::to_string(2) + extension);
    mvSLAM::FrontEndTypes::FramePtr base_frame, pair_frame;

    {
        size_t index = 1;
        const std::string filename(directory + std::to_string(index) + extension);
        mvSLAM::timestamp_us_t time(index);
        auto image = mvSLAM::load_image_grayscale(filename);
        auto frame_id = mvSLAM::FrameManager::add_frame(time, image);
        base_frame = mvSLAM::FrameManager::get_frame(frame_id);
    }

    {
        size_t index = 2;
        const std::string filename(directory + std::to_string(index) + extension);
        mvSLAM::timestamp_us_t time(index);
        auto image = mvSLAM::load_image_grayscale(filename);
        auto frame_id = mvSLAM::FrameManager::add_frame(time, image);
        pair_frame = mvSLAM::FrameManager::get_frame(frame_id);
    }

    auto p = mvSLAM::ImagePair::get_default_params();
    p.refine_structure_in_constructor = true;
    mvSLAM::ImagePair ip(base_frame, pair_frame, p);

    {
        mvSLAM::SO3 R; // use Identity
        mvSLAM::Vector3Type t{1, 0, 0}; // pure translation in X
        mvSLAM::SE3 T_2_to_1(R, t);
        ASSERT_TRUE(check_similar_SE3(T_2_to_1, ip.T_pair_to_base, tolerance));
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
