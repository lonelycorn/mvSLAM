#include <base/image.hpp>
#include <base/error.hpp>
#include <vision/visual-feature.hpp>
#include <cstdio>
#include <opencv2/highgui.hpp>
using namespace mvSLAM;

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <image_1> <image_2>\n", cmdline);
    std::printf("\tFind matching keypoints between two images.\n");
    std::printf("\t<image_1>:   filename of the first image.\n");
    std::printf("\t<image_2>:   filename of the second image.\n");
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        print_help(argv[0]);
        return static_cast<int>(mvSLAM::ApplicationErrorCode::INVALID_ARGS);
    }

    std::string image1_fn(argv[1]);
    std::string image2_fn(argv[2]);

    ImageGrayscale image1 = load_image_grayscale(image1_fn);
    ImageGrayscale image2 = load_image_grayscale(image2_fn);
    
    VisualFeature image1_vf = VisualFeature::extract(image1);
    VisualFeature image2_vf = VisualFeature::extract(image2);
    
    VisualFeatureConfig::MatchResultType match_result = 
        VisualFeature::match_visual_features(image1_vf, // base
                                             image2_vf); // new

        
    // visualization
    ImageGrayscale result_image;
    cv::drawMatches(image2, image2_vf.get_keypoints(),
                    image1, image1_vf.get_keypoints(), 
                    match_result, result_image);
    cv::imshow("Matches", result_image);
    std::printf("Press Escape to continue...\n");
    while (cv::waitKey(0) != 27);

    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}

