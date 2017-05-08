#include <base/error.hpp>
#include <base/opencv.hpp>
//#include <base/image.hpp>
#include <os/time.hpp>
#include <opencv2/highgui.hpp>

#include <sys/stat.h>
#include <sys/types.h>

#include <cstdio>
#include <string>

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <output_directory> <interval> <image_count>\n", cmdline);
    std::printf("\tTake a number of pictures at a fixed interval and save to directory.\n");
    std::printf("\t<output_directory>   directory where the images will be saved.\n");
    std::printf("\t<interval>           wait time in seconds before taking the next image.\n");
    std::printf("\t<image_count>        number of images to save.\n");
}

int main(int argc, char **argv)
{
    const std::string image_fn_extension(".jpg");
    if (argc != 4)
    {
        print_help(argv[0]);
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }

    // arg parsing
    std::string output_directory = argv[1];
    if (output_directory.back() != '/')
    {
        output_directory.push_back('/');
    }

    int result = mkdir(output_directory.c_str(), 0777);
    if (result == 0)
    {
        std::cout << "Created output directory: '" << output_directory
                << "'" << std::endl;
    }


    const uint32_t interval_ms = std::stod(argv[2]) * 1000;
    if (interval_ms <= 0)
    {
        std::cerr << "Invalid interval: " << argv[2] << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }
    /*
    else
    {
        std::cout << "Capture interval: " << interval_ms << " ms." << std::endl;
    }
    */

    const int image_count = std::stoi(argv[3]);
    if (image_count <= 0)
    {
        std::cerr << "Invalid image count: " << argv[3] << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }

    // open video capture and save
    cv::VideoCapture vc(0); // open the 1st camera device
    if (!vc.isOpened())
    {
        std::cerr << "Cannot open camera device." << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::HARDWARE_ERROR);
    }
    int image_index = 0;
    uint32_t last_capture_time_ms = mvSLAM::get_time_ms();
    while (image_index < image_count)
    {
        cv::Mat image;
        bool result = vc.read(image);
        if (!result) // disconnected
        {
            break;
        }

        if (cv::waitKey(1) == 27) // pressed Escape
        {
            break;
        }

        // save images
        auto now_ms = mvSLAM::get_time_ms();
        if (now_ms - last_capture_time_ms >= interval_ms)
        {
            const std::string image_fn = output_directory + std::to_string(image_index) + image_fn_extension;
            ++image_index;
            cv::imwrite(image_fn, image);
            last_capture_time_ms = now_ms;
        }

        // add text
        {
            const std::string text = "image index: " + std::to_string(image_index);
            const cv::Point org(50, 50);
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 1.0;
            cv::Scalar color(255, 0, 0);
            int thickness = 2;

            cv::putText(image, text, org, fontFace, fontScale, color, thickness);
        }
        // display image
        cv::imshow("Camera", image);
    }
    if (image_index < image_count)
    {
        std::cerr << "Capture terminated early. Saved" << image_index
                << "images." << std::endl;
        return static_cast<int>(mvSLAM::ApplicationErrorCode::HARDWARE_ERROR);
    }

    vc.release();
    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}
