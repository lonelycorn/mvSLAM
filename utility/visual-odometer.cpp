#include <base/error.hpp>
#include <base/image.hpp>
#include <base/debug.hpp>
#include <base/parameter-manager.hpp>
#include <os/time.hpp>
#include <visualization/visualizer.hpp>
#include <front-end/visual-odometer.hpp>
#include <front-end/camera-manager.hpp>
#include <front-end/frame-manager.hpp>

#include <string>
#include <cstdio>
#include <fstream>


static mvSLAM::Logger logger("[Application]", true);

static struct Stats
{
    uint32_t frame_total;
    uint32_t frame_tracked;

    Stats():
        frame_total(0),
        frame_tracked(0)
    {
    }

    void print()
    {
        logger.info("\n===== Visual Odometer Stats =====");
        logger.info("frame_total = ", frame_total);
        logger.info("frame_tracked = ", frame_tracked);
    }
} stats;

static void print_help(const char *cmdline)
{
    std::printf("Usage: %s <input_directory>\n", cmdline);
    std::printf("\tReplay saved trajectory.\n");
    std::printf("\t<input_directory>    directory containing all necessary information.\n");
    std::printf("\t\t<camera.config>    camera intrinsics and extrinsics.\n");
    std::printf("\t\t<system.param>     parameters of visual odometer.\n");
    std::printf("\t\t<images.txt>       list of (timestamp, filename), one row for each relevant image.\n");
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        print_help(argv[0]);
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }

    const std::string directory(argv[1]);

    // read param from config file
    {
        const std::string filename = directory + "system.param";
        mvSLAM::ParameterManager::load_from_file(filename);
    }
    mvSLAM::VisualOdometer::Params vo_params = mvSLAM::VisualOdometer::get_default_params();
    mvSLAM::VisualOdometer vo(vo_params);

    // load camera config
    {
        const std::string filename = directory + "/camera.config";
        mvSLAM::CameraManager::load_from_file(filename);
    }

    //  set up visualization
    mvSLAM::Visualizer viewer("Visual Odometer", mvSLAM::Visualizer::get_default_params());

    const std::string image_txt_filename = directory + "/image.txt";
    std::fstream in(image_txt_filename, std::ios_base::in);
    mvSLAM::timestamp_us_t capture_time = 0;
    bool vo_initialized = false;
    while (!in.eof())
    {
        ++capture_time;
        logger.info("===== t = ", capture_time, "=====");

        std::string filename;
        in >> filename;
        if (filename.size() == 0)
        {
            break;
        }
        filename = directory + "/" + filename;
        logger.info("read image file :'", filename, "'");
        auto image = mvSLAM::load_image_grayscale(filename);
        auto frame_id = mvSLAM::FrameManager::add_frame(capture_time, image);
        logger.info("frame id = ", frame_id);

        bool success = vo.add_frame_by_id(frame_id);

        ++stats.frame_total;
        if (success)
        {
            logger.info("body pose =\n", vo.get_body_pose());
            viewer.set_camera_pose(stats.frame_total, vo.get_camera_pose());
            viewer.set_point_cloud(stats.frame_total, vo.get_tracked_points());
            ++stats.frame_tracked;
        }

        if (!vo_initialized && vo.initialized())
        {
            vo_initialized = true;
            logger.info("visual odometer initialized");
        }
        else if (vo_initialized && !vo.initialized())
        {
            vo_initialized = false;
            logger.error("visual odometer failed to track");
        }
    }

    stats.print();

    while (!viewer.is_window_closed())
    {
        mvSLAM::sleep_ms(1000);
    }

    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}
