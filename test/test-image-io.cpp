#include <base/image.hpp>
#include <base/error.hpp>
#include <vision/io.hpp>
#include <opencv2/highgui.hpp>
#include <cstdio>
using namespace mvSLAM;

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <input_rgb_image> <output_gray_image>\n", cmdline);
    std::printf("\tload and save image.\n");
    std::printf("\t<input_rgb_image>:    filename of the input color image.\n");
    std::printf("\t<output_gray_image>:  filename of the output grayscale image.\n");
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        print_help(argv[0]);
        return mvSLAM::ApplicationErrorCode::AEC_INVALID_ARGS;
    }
    std::string input_fn(argv[1]);
    std::string output_fn(argv[2]);
    ImageRGB image_rgb = load_image_rgb(input_fn);
    if (!save_image_grayscale(output_fn, image_rgb))
    {
        std::printf("Unable to save grayscale image %s.\n", output_fn.c_str());
        return ApplicationErrorCode::AEC_IO;
    }
    ImageGrayscale image_gray = load_image_grayscale(output_fn);
    
    // visualization
    cv::imshow("Grayscale", image_gray);
    cv::imshow("Color", image_rgb);
    std::printf("Press any key to continue...\n");
    cv::waitKey(0);

    return mvSLAM::ApplicationErrorCode::AEC_NONE;
}
