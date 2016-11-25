#include <pcl/visualization/cloud_viewer.h>
#include <base/error.hpp>
#include <base/image.hpp>
#include <base/visualization.hpp>
#include <vision/io.hpp>
#include <vision/visual-feature.hpp>
#include <vision/sfm.hpp>
#include <cstdio>
#include <iostream>
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
    std::cout<<"scaled translation="<<std::endl;
    std::cout<<pose2in1_scaled.translation()<<std::endl;
    std::cout<<"scaled rotation="<<std::endl;
    std::cout<<pose2in1_scaled.rotation().get_matrix()<<std::endl;
    /*
    std::printf("pointsin1_scaled =\n");
    for (const auto &p : pointsin1_scaled)
    {
        std::cout<< p.x() <<", "<<p.y()<<","<<p.z()<<std::endl;
    }
    */
        
    // visualize points
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
        mvSLAM::Point3D_to_PointCloud(pointsin1_scaled);

    pcl::visualization::PCLVisualizer viewer("3D Reconstruction");
    viewer.addPointCloud(pc, "triangulated points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            1, "triangulated points");

    // visualize camera poses
    mvSLAM::add_camera_representation(mvSLAM::SE3(), "1", viewer);
    mvSLAM::add_camera_representation(pose2in1_scaled, "2", viewer);

    mvSLAM::initialize_visualizer(viewer);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }

    return mvSLAM::ApplicationErrorCode::AEC_NONE;
}
