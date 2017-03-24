#include <cassert>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

#include <base/debug.hpp>
#include <back-end/graph.hpp>
#include <base/gtsam.hpp>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace mvSLAM
{
/// Meta data for nodes
struct MetaDataNode
{
    BackEndTypes::NodeId id;

    std::unordered_set<BackEndTypes::EdgeId> attached_edge_id;

    explicit MetaDataNode(BackEndTypes::NodeId id_):
        id(id_), attached_edge_id()
    {
    }

    MetaDataNode(const MetaDataNode &) = default;
    MetaDataNode(MetaDataNode &&) = default;

    void register_edge(BackEndTypes::EdgeId edge_id)
    {
        auto p = attached_edge_id.insert(edge_id);
        (void) p;
        assert(p.second);
    }
};

/// meta data for edges
struct MetaDataEdge
{
    BackEndTypes::EdgeId id;
    BackEndTypes::NodeId src_node_id;
    BackEndTypes::NodeId dst_node_id;
    MetaDataEdge(BackEndTypes::EdgeId id_,
                 BackEndTypes::NodeId src_,
                 BackEndTypes::NodeId dst_):
        id(id_), src_node_id(src_), dst_node_id(dst_)
    {
    }
    MetaDataEdge(const MetaDataEdge &) = default;
    MetaDataEdge(MetaDataEdge &&) = default;
};

/// Data container for Graph
class GraphImpl
{
public:

    GraphImpl():
        id(Id::INVALID),
        origin_node_id(Id::INVALID),
        meta_data_node(),
        meta_data_edge(),
        graph(),
        values()
    {
    }

    ~GraphImpl()
    {
    }

    static constexpr ScalarType GRAPH_ANCHOR_STDDEV = 1e-4;

    BackEndTypes::GraphId id;
    BackEndTypes::NodeId origin_node_id;

    std::unordered_map<BackEndTypes::NodeId, MetaDataNode> meta_data_node;
    std::unordered_map<BackEndTypes::EdgeId, MetaDataEdge> meta_data_edge;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
};
constexpr ScalarType GraphImpl::GRAPH_ANCHOR_STDDEV;


Graph::Graph(const BackEndTypes::PoseNodeValue &origin):
    m_impl(new GraphImpl())
{
    // register the graph
    m_impl->id = BackEndTypes::generate_graph_id();

    // register the origin node
    m_impl->origin_node_id = add_pose_node(origin);

    // anchor the origin node
    gtsam::Key key = m_impl->origin_node_id;
    gtsam::Pose3 pose = SE3_to_Pose3(origin);
    gtsam::Vector6 stddev;
    stddev << GraphImpl::GRAPH_ANCHOR_STDDEV,
              GraphImpl::GRAPH_ANCHOR_STDDEV,
              GraphImpl::GRAPH_ANCHOR_STDDEV,
              GraphImpl::GRAPH_ANCHOR_STDDEV,
              GraphImpl::GRAPH_ANCHOR_STDDEV,
              GraphImpl::GRAPH_ANCHOR_STDDEV;
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(stddev);
    auto prior = gtsam::PriorFactor<gtsam::Pose3>(key, pose, noise);
    m_impl->graph.push_back(prior);
}

Graph::~Graph()
{
    delete m_impl;
}

BackEndTypes::BackEndTypes::NodeId
Graph::add_pose_node(const BackEndTypes::PoseNodeValue &P)
{
    BackEndTypes::BackEndTypes::NodeId node_id = BackEndTypes::generate_node_id();
    assert(m_impl->meta_data_node.count(node_id) == 0);

    // register the node
    m_impl->meta_data_node.emplace(node_id, MetaDataNode(node_id));

    // add to value
    gtsam::Pose3 p = SE3_to_Pose3(P);
    m_impl->values.insert(node_id, p);

    return node_id;
}

BackEndTypes::BackEndTypes::EdgeId 
Graph::add_transformation_edge(BackEndTypes::BackEndTypes::NodeId src,
                               BackEndTypes::BackEndTypes::NodeId dst,
                               const BackEndTypes::TransformationEdgeValue &T)
{
    assert(m_impl->meta_data_node.count(src) == 1);
    assert(m_impl->meta_data_node.count(dst) == 1);

    BackEndTypes::BackEndTypes::EdgeId edge_id = BackEndTypes::generate_edge_id();
    assert(m_impl->meta_data_edge.count(edge_id) == 0);
    
    // register the edge
    m_impl->meta_data_edge.emplace(edge_id, MetaDataEdge(edge_id, src, dst));
    m_impl->meta_data_node.at(src).register_edge(edge_id);
    m_impl->meta_data_node.at(dst).register_edge(edge_id);

    // add to graph
    auto mean = SE3_to_Pose3(T.mean());
    auto covar = gtsam::noiseModel::Gaussian::Covariance(T.covar());
    auto factor = gtsam::BetweenFactor<gtsam::Pose3>(src, dst, mean, covar);
    m_impl->graph.push_back(factor);

    return edge_id;
}

bool
Graph::has_node(BackEndTypes::NodeId node_id) const
{
    return ((Id::INVALID != node_id) && (m_impl->meta_data_node.count(node_id) == 1));
}

bool
Graph::has_edge(BackEndTypes::EdgeId edge_id) const
{
    return ((Id::INVALID != edge_id) && (m_impl->meta_data_edge.count(edge_id) == 1));
}

BackEndTypes::PoseNodeValue
Graph::get_pose_node_value(BackEndTypes::BackEndTypes::NodeId node_id) const
{
    assert(m_impl->meta_data_node.count(node_id) == 1); 
    auto p = m_impl->values.at<gtsam::Pose3>(node_id);
    return Pose3_to_SE3(p);
}

std::unordered_map<BackEndTypes::BackEndTypes::NodeId, BackEndTypes::PoseNodeValue>
Graph::get_all_pose_node_value() const
{
    std::unordered_map<BackEndTypes::BackEndTypes::NodeId, BackEndTypes::PoseNodeValue> result;
    for (const auto &entry : m_impl->meta_data_node)
    {
        auto node_id = entry.first;
        auto p = m_impl->values.at<gtsam::Pose3>(node_id);
        result[node_id] = Pose3_to_SE3(p);
    }
    return result;
}

bool
Graph::reconcile_with(Graph &other)
{
    assert(false && "NOT implemented yet");
    /*
    if (succesfully_reconciled)
    {
        other.clear();
        return true;
    }
    */
    return false;
}

BackEndTypes::BackEndTypes::GraphId
Graph::get_id() const
{
    return m_impl->id;
}

BackEndTypes::BackEndTypes::NodeId
Graph::get_origin_node_id() const
{
    return m_impl->origin_node_id;
}

void
Graph::print(const char *s) const
{
    m_impl->graph.print(s);
    m_impl->values.print(s);
}

class GraphOptimizerImpl
{
public:
    GraphOptimizerImpl(const gtsam::NonlinearFactorGraph &g,
                       const gtsam::Values &v):
        optimizer(g, v),
        values(v)
    {
        // gtsam::LevenbergMarquardtOptimizer makes a copy of the graph...
    }
    ~GraphOptimizerImpl()
    {
    }
    gtsam::LevenbergMarquardtOptimizer optimizer;
    gtsam::Values values;
};

GraphOptimizer::GraphOptimizer(const Graph &g):
    m_impl(new GraphOptimizerImpl(g.m_impl->graph, g.m_impl->values))
{
}

GraphOptimizer::~GraphOptimizer()
{
    delete m_impl;
}

void
GraphOptimizer::optimize()
{
    m_impl->values = m_impl->optimizer.optimize();
}

bool
GraphOptimizer::update_graph(Graph &g)
{
    // update graph values without adding new elements
    g.m_impl->values.update(m_impl->values);
    return true;
}

}
