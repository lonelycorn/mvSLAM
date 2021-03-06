#include "unit-test.hpp"
#include <string>
#include <os/time.hpp>
#include <front-end/frame-manager.hpp>
#include <iostream>
#include <set>

using namespace unit_test;

UNIT_TEST(frame_manager)
{
    const mvSLAM::ScalarType tolerance = 1e-1;
    const mvSLAM::ScalarType fps = 10;
    const std::string directory("../data/tsukuba/");
    const std::string extension("jpg");
    std::set<mvSLAM::FrontEndTypes::FrameId> allocated_frame_id;

    // add frames
    for (int i = 1; i <= 5; ++i)
    {
        std::string filename = directory + std::to_string(i) + "." + extension;
        mvSLAM::timestamp_us_t time = i * MS_PER_S / fps;

        auto image = mvSLAM::load_image_grayscale(filename);
        // check if successfully read the image
        ASSERT_TRUE((image.rows > 0) && (image.cols > 0));

        auto frame_id = mvSLAM::FrameManager::add_frame(time, image);
        ASSERT_TRUE(frame_id != mvSLAM::Id::INVALID);
        ASSERT_TRUE(allocated_frame_id.count(frame_id) == 0);
        allocated_frame_id.insert(frame_id);

        auto frame_ptr = mvSLAM::FrameManager::get_frame(frame_id);
        ASSERT_TRUE(frame_ptr->id == frame_id);
        ASSERT_TRUE(frame_ptr->capture_time == time);
        ASSERT_TRUE(frame_ptr->visual_feature.size() > 0);


        ASSERT_TRUE(mvSLAM::FrameManager::size() == static_cast<size_t>(i));
        std::cout<<"capture time: " << time <<", current fps: " << mvSLAM::FrameManager::get_fps()<<std::endl;
    }

    ASSERT_EQUAL(mvSLAM::FrameManager::get_fps(), fps, tolerance);

    // erase frames
    size_t count = mvSLAM::FrameManager::size();
    for (auto it = allocated_frame_id.begin(); it != allocated_frame_id.end(); ++it)
    {
        bool success = mvSLAM::FrameManager::erase_frame(*it);
        ASSERT_TRUE(success);
        --count;
        ASSERT_TRUE(count == mvSLAM::FrameManager::size());
    }
    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
