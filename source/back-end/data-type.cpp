#include <back-end/data-type.hpp>
#include <atomic>

namespace mvSLAM
{
BackEndTypes::NodeId
BackEndTypes::generate_node_id()
{
    static std::atomic<NodeId> next_id(0);
    return next_id.fetch_add(1);
}

BackEndTypes::EdgeId
BackEndTypes::generate_edge_id()
{
    static std::atomic<EdgeId> next_id(0);
    return next_id.fetch_add(1);
}

BackEndTypes::GraphId
BackEndTypes::generate_graph_id()
{
    static std::atomic<GraphId> next_id(0);
    return next_id.fetch_add(1);
}
}
