#pragma once

#include <system-config.hpp>
#include <base/space.hpp>

namespace mvSLAM
{
class BackEndTypes
{
public:
    using NodeId = Id::Type;
    using EdgeId = Id::Type;
    using GraphId = Id::Type;


    static NodeId generate_node_id();
    static EdgeId generate_edge_id();
    static GraphId generate_graph_id();

    using TransformationEdgeValue = TransformationEstimate;
    using PoseNodeValue = Transformation;
};

}
