#include <pcl/visualization/cloud_viewer.h>
#include <base/error.hpp>
#include <base/image.hpp>
#include <vision/io.hpp>
#include <vision/visual-feature.hpp>
#include <vision/sfm.hpp>
#include <cstdio>
#include <vector>

void print_help(const char *cmdline)
{
    std::printf("Usage: %s <image_1> <image_2> <intrinsics>\n", cmdline);
    std::printf("\tReconstruct scene using two images.\n");
    std::printf("\t<image_1>:       filename of the first image.\n");
    std::printf("\t<image_2>:       filename of the second image.\n");
    std::printf("\t<intrinsics>:    filename of the camera intrinsics.\n");
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        print_help(argv[0]);
        return mvSLAM::ApplicationErrorCode::AEC_INVALID_ARGS;
    }

    std::string image1_fn(argv[1]);
    std::string image2_fn(argv[2]);
    std::string camera_intrinsics_fn(argv[3]);

    mvSLAM::ImageGrayscale image1 = mvSLAM::load_image_grayscale(image1_fn);
    mvSLAM::ImageGrayscale image2 = mvSLAM::load_image_grayscale(image2_fn);
    mvSLAM::CameraIntrinsics K;
    if (!mvSLAM::load_camera_intrinsics(camera_intrinsics_fn, K))
    {
        std::printf("Unable to read camera intrinsics.\n");
        return mvSLAM::ApplicationErrorCode::AEC_IO;
    }
    
    mvSLAM::Pose pose2in1_scaled;
    std::vector<mvSLAM::Point3D> pointsin1_scaled;
    if (!reconstruct_scene(image1,
                           image2,
                           K,
                           pose2in1_scaled,
                           pointsin1_scaled))
    {
        std::printf("Reconstruction failed.\n");
        return mvSLAM::ApplicationErrorCode::AEC_BAD_DATA;
    }
        
    // visualization
    pcl::visualization::CloudViewer viewer("3D Reconstruction");
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    size_t point_count = pointsin1_scaled.size();
    for (size_t i = 0; i < point_count; ++i)
    {
        const auto &p = pointsin1_scaled[i];
        pc->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }

    viewer.showCloud(pc);
    while (!viewer.wasStopped())
    {
        // do some IO here to avoid busy waiting.
        std::string s;
        std::getline(std::cin, s);
    }

    return mvSLAM::ApplicationErrorCode::AEC_NONE;
}
