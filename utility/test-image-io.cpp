#include <base/image.hpp>
#include <base/error.hpp>
#include <vision/io.hpp>
#include <opencv2/highgui.hpp>
#include <cstdio>
using namespace mvSLAM;

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <input_rgb_image>>\n", cmdline);
    std::printf("\tload and save (overwrite) the rgb image, and show the grayscale version.\n");
    std::printf("\t<input_rgb_image>:    filename of the input color image.\n");
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        print_help(argv[0]);
        return mvSLAM::ApplicationErrorCode::AEC_INVALID_ARGS;
    }
    std::string input_fn(argv[1]);
    std::string output_fn(input_fn);
    ImageRGB image_rgb = load_image_rgb(input_fn);
    if (!save_image_rgb(output_fn, image_rgb))
    {
        std::printf("Unable to save color image %s.\n", output_fn.c_str());
        return ApplicationErrorCode::AEC_IO;
    }
    ImageGrayscale image_gray = load_image_grayscale(input_fn);
    
    // visualization
    cv::imshow("Grayscale", image_gray);
    cv::imshow("Color", image_rgb);
    std::printf("Press Escape to continue...\n");
    while (cv::waitKey(0) != 27);

    return mvSLAM::ApplicationErrorCode::AEC_NONE;
}