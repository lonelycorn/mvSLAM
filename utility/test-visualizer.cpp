#include <iostream>
#include <sstream>
#include <random>

#include <system-config.hpp>
#include <base/error.hpp>
#include <base/space.hpp>
#include <visualization/data-types.hpp>
#include <visualization/visualizer.hpp>

mvSLAM::VisualizationTypes::PointCloud
get_random_point_cloud(const int point_count)
{
    static std::default_random_engine generator;
    static std::normal_distribution<mvSLAM::ScalarType> normal_dist(0.0, 1.0);

    mvSLAM::VisualizationTypes::PointCloud result;
    for (int i = 0; i < point_count; ++i)
    {
        mvSLAM::ScalarType x = normal_dist(generator);
        mvSLAM::ScalarType y = normal_dist(generator);
        mvSLAM::ScalarType z = normal_dist(generator);

        result.emplace(i, mvSLAM::Vector3Type(x, y, z));
    }
    return result;
}

mvSLAM::SE3
get_random_camera_pose(mvSLAM::ScalarType x)
{
    static std::default_random_engine generator;
    static std::normal_distribution<mvSLAM::ScalarType> normal_dist(0.0, 1.0);

    mvSLAM::Vector6Type se3;
    se3[0] = x;
    for (int i = 1; i < 6; ++i)
    {
        se3[i] = normal_dist(generator);
    }

    return mvSLAM::SE3::exp(se3);
}


int main()
{
    const auto params = mvSLAM::Visualizer::get_default_params();
    mvSLAM::Visualizer viewer("whooja", params);

    for (int i = 0; i < 8; ++i)
    {
        std::cout<<"===== i = "<<i<<" ====="<<std::endl;

        std::cout<<"generating random point cloud"<<std::endl;
        const mvSLAM::VisualizationTypes::PointCloudId pid = i / 2;
        const int point_count = 500;
        const auto pc = get_random_point_cloud(point_count);
        viewer.set_point_cloud(pid, pc);

        std::cout<<"generating random camera pose"<<std::endl;
        const mvSLAM::VisualizationTypes::CameraId cid = i / 2;
        const auto T = get_random_camera_pose(static_cast<mvSLAM::ScalarType>(i / 4.0));
        viewer.set_camera_pose(cid, T, static_cast<mvSLAM::ScalarType>(i / 3.0 + 0.5));

        mvSLAM::sleep_ms(2000);
        //std::getchar();
    }
    return static_cast<int>(mvSLAM::ApplicationErrorCode::NONE);
}
