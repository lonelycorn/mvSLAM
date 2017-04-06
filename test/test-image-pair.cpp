#include "unit-test.hpp"
#include <string>
#include <front-end/frame-manager.hpp>
#include <front-end/image-pair.hpp>
#include <iostream>
#include <set>

using namespace unit_test;

UNIT_TEST(image_pair)
{
    const std::string directory("../data/tsukuba/");
    const std::string extension(".jpg");
    const std::string pair_frame_filename(directory + std::to_string(2) + extension);
    mvSLAM::FrontEndTypes::FramePtr base_frame, pair_frame;

    {
        size_t index = 1;
        const std::string filename(directory + std::to_string(index) + extension);
        mvSLAM::timestamp_us_t time(index);
        auto image = mvSLAM::load_image_grayscale(filename);
        auto frame_id = mvSLAM::FrameManager::get_instance().add_frame(time, image);
        base_frame = mvSLAM::FrameManager::get_instance().get_frame(frame_id);
    }

    {
        size_t index = 2;
        const std::string filename(directory + std::to_string(index) + extension);
        mvSLAM::timestamp_us_t time(index);
        auto image = mvSLAM::load_image_grayscale(filename);
        auto frame_id = mvSLAM::FrameManager::get_instance().add_frame(time, image);
        pair_frame = mvSLAM::FrameManager::get_instance().get_frame(frame_id);
    }

    mvSLAM::ImagePair ip(base_frame, pair_frame);

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
