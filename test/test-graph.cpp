#include <cmath>
#include <vector>
#include <unordered_map>
#include <iostream>

#include <back-end/graph.hpp>
#include "unit-test.hpp"
#include "unit-test-helper.hpp"

using namespace unit_test;

//#define DEBUG_OUTPUT

// check if ID's are monotonically increasing
UNIT_TEST(id_generators)
{
    constexpr size_t id_count = 10;
    
    std::vector<mvSLAM::BackEndTypes::NodeId> generated_node_id;
    for (size_t i = 0; i < id_count; ++i)
    {
        auto id = mvSLAM::BackEndTypes::generate_node_id();
        ASSERT_TRUE(id != mvSLAM::Id::INVALID);
        auto monotonically_increasing = true;
        for (auto j : generated_node_id)
        {
            if (j <= id)
            {
                monotonically_increasing = false;
                break;
            }
        }
        ASSERT_TRUE(monotonically_increasing);
    }
    
    std::vector<mvSLAM::BackEndTypes::EdgeId> generated_edge_id;
    for (size_t i = 0; i < id_count; ++i)
    {
        auto id = mvSLAM::BackEndTypes::generate_edge_id();
        ASSERT_TRUE(id != mvSLAM::Id::INVALID);
        auto monotonically_increasing = true;
        for (auto j : generated_edge_id)
        {
            if (j <= id)
            {
                monotonically_increasing = false;
                break;
            }
        }
        ASSERT_TRUE(monotonically_increasing);
    }

    std::vector<mvSLAM::BackEndTypes::GraphId> generated_graph_id;
    for (size_t i = 0; i < id_count; ++i)
    {
        auto id = mvSLAM::BackEndTypes::generate_graph_id();
        ASSERT_TRUE(id != mvSLAM::Id::INVALID);
        auto monotonically_increasing = true;
        for (auto j : generated_graph_id)
        {
            if (j <= id)
            {
                monotonically_increasing = false;
                break;
            }
        }
        ASSERT_TRUE(monotonically_increasing);
    }
    PASS();
}

UNIT_TEST(trivial)
{
    constexpr mvSLAM::ScalarType TOLERANCE(0.01);

    // default value: Identity
    mvSLAM::Transformation origin_in_world;
    mvSLAM::Graph graph(origin_in_world);


    mvSLAM::Vector6Type true_se3;
    true_se3 << 1, 0, 0, // translation part
                0, 0, 0;
    mvSLAM::Vector6Type guess_se3 = true_se3 * 1.2;

    mvSLAM::Transformation pose_true = mvSLAM::SE3::exp(true_se3) * origin_in_world;
    mvSLAM::Transformation guess_pose = mvSLAM::SE3::exp(guess_se3) * origin_in_world;

    // isotropic noise
    const mvSLAM::ScalarType transformation_noise_stddev(0.01);
    mvSLAM::TransformationUncertainty uncertainty(
            mvSLAM::TransformationUncertainty::Identity() * transformation_noise_stddev);

    mvSLAM::BackEndTypes::NodeId src = graph.get_origin_node_id();
    mvSLAM::BackEndTypes::NodeId dst = graph.add_pose_node(guess_pose);
    mvSLAM::TransformationEstimate edge(mvSLAM::SE3::exp(true_se3), uncertainty);

    graph.add_transformation_edge(src, dst, edge);

    mvSLAM::GraphOptimizer optimizer(graph);
    optimizer.optimize();
    
    // make sure optimizer is working on a copy
    ASSERT_TRUE(check_similar_SE3(graph.get_pose_node_value(dst),
                                  guess_pose,
                                  TOLERANCE));

    // check if GraphOptimizer::update_graph() works
    optimizer.update_graph(graph);
    //std::cout<<"updated graph \n"<<graph.get_pose_node_value(dst)<<std::endl;
    ASSERT_TRUE(check_similar_SE3(graph.get_pose_node_value(dst),
                                  pose_true,
                                  TOLERANCE));

    PASS();
}

UNIT_TEST(planar_triangle)
{
    constexpr mvSLAM::ScalarType TOLERANCE(0.03);

    // ID of all created nodes
    std::vector<mvSLAM::BackEndTypes::NodeId> created_node_id;
    std::unordered_map<mvSLAM::BackEndTypes::NodeId, mvSLAM::Transformation> pose_true;
    // a third of a circle
    mvSLAM::SE3 motion_true(mvSLAM::SO3(0.0, 0.0, M_PI*2.0/3.0), mvSLAM::Vector3Type{1, 0, 0});

    // initialize trajectory
    mvSLAM::SE3 origin_in_world; // default value: Identity
    mvSLAM::Graph graph(origin_in_world);
    mvSLAM::BackEndTypes::NodeId last_node_id = graph.get_origin_node_id();
    mvSLAM::SE3 last_node_pose = origin_in_world;
    mvSLAM::SE3 last_node_pose_dead_reckoning = origin_in_world;
    created_node_id.push_back(last_node_id);
    pose_true[last_node_id] = origin_in_world;

    // isotropic noise
    const mvSLAM::ScalarType transformation_noise_stddev(0.01);
    mvSLAM::TransformationUncertainty uncertainty(
            mvSLAM::TransformationUncertainty::Identity() * mvSLAM::sqr(transformation_noise_stddev));

    // build the graph with dead-reckoning
    for (size_t i = 0; i < 3; ++i)
    {
        // measurement noise
        mvSLAM::SE3 delta = get_SE3_sample(transformation_noise_stddev);

        // actual measurement
        mvSLAM::SE3 mean = delta * motion_true;
        mvSLAM::TransformationEstimate constraint(mean , uncertainty);

        last_node_pose_dead_reckoning = mean * last_node_pose_dead_reckoning;
        last_node_pose = motion_true * last_node_pose;

        auto node_id = graph.add_pose_node(last_node_pose_dead_reckoning);
        auto edge_id = graph.add_transformation_edge(created_node_id.back(), node_id, constraint);
        (void) edge_id;
        
        created_node_id.push_back(node_id);
        pose_true[node_id] = last_node_pose;
    }

    // close the loop with identification
    {
        mvSLAM::TransformationEstimate constraint(mvSLAM::SE3(), uncertainty);
        auto edge_id = graph.add_transformation_edge(created_node_id.back(), created_node_id[0], constraint);
        (void) edge_id;
    }

#ifdef DEBUG_OUTPUT
    graph.print("===== before optimization =====");
#endif
    // optimize
    mvSLAM::GraphOptimizer optimizer(graph);
    optimizer.optimize();
    optimizer.update_graph(graph);
#ifdef DEBUG_OUTPUT
    graph.print("===== after optimization =====");
#endif
    
    for (auto node_id : created_node_id)
    {
        auto pose1 = graph.get_pose_node_value(node_id);
        auto pose2 = pose_true[node_id]; 
#ifdef DEBUG_OUTPUT
        std::cout<<"===== node "<<node_id<<" =====\ntrue pose\n"
                 <<pose2<<"\nestimated pose\n"<<pose1<<std::endl;
#endif
        ASSERT_TRUE(check_similar_SE3(pose1, pose2, TOLERANCE));
    }

    PASS();
}

int main()
{
    RUN_ALL_TESTS();
}

