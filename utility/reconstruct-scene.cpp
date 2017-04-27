#include <pcl/visualization/cloud_viewer.h>
#include <base/error.hpp>
#include <base/image.hpp>
#include <base/visualization.hpp>
#include <vision/visual-feature.hpp>
#include <vision/camera.hpp>
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
    std::printf("\t<max_dist>:      max distance between matched features.\n");
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        print_help(argv[0]);
        return mvSLAM::ApplicationErrorCode::AEC_INVALID_ARGS;
    }

    std::string image1_fn(argv[1]);
    std::string image2_fn(argv[2]);
    std::string camera_intrinsics_fn(argv[3]);
    mvSLAM::ScalarType max_dist = std::stoi(std::string(argv[4]));

    // input
    mvSLAM::ImageGrayscale image1 = mvSLAM::load_image_grayscale(image1_fn);
    mvSLAM::ImageGrayscale image2 = mvSLAM::load_image_grayscale(image2_fn);
    auto image1_vf = mvSLAM::VisualFeature::extract(image1);
    auto image2_vf = mvSLAM::VisualFeature::extract(image2);
    auto matched_vf_pair = mvSLAM::VisualFeature::match_and_filter_visual_features(
        image1_vf, image2_vf, max_dist);
    mvSLAM::PinholeCamera camera(camera_intrinsics_fn);

    // output
    mvSLAM::Transformation pose2in1_scaled;
    std::vector<mvSLAM::Point3> pointsin1_scaled;
    std::vector<size_t> point_indexes;
    if (!sfm_solve(matched_vf_pair.first.get_image_points(),
                   matched_vf_pair.second.get_image_points(),
                   camera.get_intrinsics(),
                   pose2in1_scaled,
                   pointsin1_scaled,
                   point_indexes))
    {
        std::printf("Reconstruction failed.\n");
        return mvSLAM::ApplicationErrorCode::AEC_BAD_DATA;
    }
    std::cout<<"camera intrinsics:\n"<<camera.get_intrinsics()<<std::endl;
    std::cout<<"camera extrinsics:\n"<<camera.get_extrinsics()<<std::endl;
    std::cout<<"scaled transformation =\n"<<pose2in1_scaled<<std::endl;
    std::printf("pointsin1_scaled =\n");
    for (const auto &p : pointsin1_scaled)
    {
        std::cout<< p.x() <<", "<<p.y()<<", "<<p.z()<<std::endl;
    }
#if 1
    // visualize points
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc =
        mvSLAM::Point3_to_PointCloud(pointsin1_scaled);

    pcl::visualization::PCLVisualizer viewer("3D Reconstruction");
    viewer.addPointCloud(pc, "triangulated points");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            1, "triangulated points");

    // visualize camera poses
    mvSLAM::add_camera_representation(mvSLAM::SE3(), "1", viewer);
    mvSLAM::add_camera_representation(pose2in1_scaled, "2", viewer, true);

    mvSLAM::initialize_visualizer(viewer);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
#endif    
    return mvSLAM::ApplicationErrorCode::AEC_NONE;
}
