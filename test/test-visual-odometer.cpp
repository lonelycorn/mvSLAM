#include "unit-test.hpp"
#include "unit-test-helper.hpp"

#include <front-end/visual-odometer.hpp>
#include <front-end/camera-manager.hpp>
#include <front-end/frame-manager.hpp>

#include <string>
#include <iostream>
#include <set>

using namespace unit_test;

UNIT_TEST(visual_odometer_initialization)
{
    const std::string directory("../data/tsukuba/");
    const std::string extension("jpg");

    mvSLAM::VisualOdometer::Params vo_params = mvSLAM::VisualOdometer::get_default_params();
    mvSLAM::VisualOdometer vo(vo_params);

    mvSLAM::CameraManager::load_from_file("../data/tsukuba/camera.config");

    // first frame
    {
        std::string filename = directory + std::to_string(1) + "." + extension;
        mvSLAM::timestamp_us_t time = 1;

        auto image = mvSLAM::load_image_grayscale(filename);
        ASSERT_TRUE((image.rows > 0) && (image.cols > 0)); // check if successfully read the image

        auto frame_id = mvSLAM::FrameManager::get_instance().add_frame(time, image);
        ASSERT_TRUE(frame_id != mvSLAM::Id::INVALID);

        mvSLAM::Transformation T;
        bool success = vo.add_frame_by_id(frame_id, T);

        ASSERT_TRUE(!success);
    }

    // second frame
    {
        std::string filename = directory + std::to_string(2) + "." + extension;
        mvSLAM::timestamp_us_t time = 2;

        auto image = mvSLAM::load_image_grayscale(filename);
        ASSERT_TRUE((image.rows > 0) && (image.cols > 0)); // check if successfully read the image

        auto frame_id = mvSLAM::FrameManager::get_instance().add_frame(time, image);
        ASSERT_TRUE(frame_id != mvSLAM::Id::INVALID);

        mvSLAM::Transformation T;
        bool success = vo.add_frame_by_id(frame_id, T);

        ASSERT_TRUE(success);
    }

    PASS();
}

UNIT_TEST(visual_odometer_tracking)
{
    const mvSLAM::ScalarType tolerance(1e-3);
    const std::string directory("../data/tsukuba/");
    const std::string extension("jpg");
    const size_t total_frame_count = 5;

    mvSLAM::CameraManager::load_from_file("../data/tsukuba/camera.config");

    mvSLAM::VisualOdometer::Params vo_params = mvSLAM::VisualOdometer::get_default_params();
    //vo_params.max_match_inlier_distance = 15;
    mvSLAM::VisualOdometer vo(vo_params);

    for (size_t i = 0; i < total_frame_count; ++i)
    {
        std::string filename = directory + std::to_string(i+1) + "." + extension;
        mvSLAM::timestamp_us_t time = i;

        auto image = mvSLAM::load_image_grayscale(filename);
        ASSERT_TRUE((image.rows > 0) && (image.cols > 0)); // check if successfully read the image

        auto frame_id = mvSLAM::FrameManager::get_instance().add_frame(time, image);
        ASSERT_TRUE(frame_id != mvSLAM::Id::INVALID);

        mvSLAM::Transformation pose;
        bool result = vo.add_frame_by_id(frame_id, pose);
        if (i == 0)
        {
            ASSERT_TRUE(!result);
            ASSERT_TRUE(check_similar_SE3(pose, mvSLAM::Transformation(), tolerance));
        }
        else
        {
#ifdef DEBUG_OUTPUT
            std::cout<<"========== i = "<<i<<" ============"<<std::endl;
            std::cout<<"result = "<<result<<", pose = \n"<<pose<<std::endl;
            std::cout<<"==================================="<<std::endl;
#endif // DEBUG_OUTPUT
            ASSERT_TRUE(result);

            mvSLAM::Vector3Type translation;
            translation << i, 0, 0;
            mvSLAM::SO3 rotation;
            mvSLAM::Transformation T(rotation, translation);
            ASSERT_TRUE(check_similar_SE3(pose, T, i * tolerance));
        }
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}
