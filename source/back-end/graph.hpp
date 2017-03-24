#pragma once

#include <unordered_map>

#include <back-end/data-type.hpp>

namespace mvSLAM
{
/// Forward declaration; the data container of @ref Graph.
class GraphImpl;
/// Forward declaration; the optimizer of @ref Graph
class GraphOptimizer; 

/// A collection of connected nodes and edges.
class Graph
{
public:

    Graph(const BackEndTypes::PoseNodeValue &origin);
    ~Graph();

    /** Add a new pose node to the graph.
     * NOTE: this is the only supported node type right now.
     * @return id of the new node.
     */
    BackEndTypes::NodeId
        add_pose_node(const BackEndTypes::PoseNodeValue &P); 

    /** Add a transformation edge to the graph.
     * NOTE: this is the only supported edge type right now.
     * @param [in] src  id of the source node.
     * @param [in] dst  id of the destination node.
     * @param [in] e    estimate of the transformation from @p src to @dst.
     * @return id of the new edge.
     */
    BackEndTypes::EdgeId 
        add_transformation_edge(BackEndTypes::NodeId src,
                                BackEndTypes::NodeId dst,
                                const BackEndTypes::TransformationEdgeValue &T);
    bool
        has_node(BackEndTypes::NodeId node_id) const;

    bool
        has_edge(BackEndTypes::EdgeId edge_id) const;

    /** Get the value of the specified pose node.
     */
    BackEndTypes::PoseNodeValue
        get_pose_node_value(BackEndTypes::NodeId node_id) const;

    /** Get the values of all pose nodes.
     */
    std::unordered_map<BackEndTypes::NodeId, BackEndTypes::PoseNodeValue>
        get_all_pose_node_value() const;

    /** Reconcile with another graph instance.
     * This is useful when merging two Graphs together.
     * @param [in, out] other   the other graph instace to reconcile with.
     *      will be invalidated upon success.
     * @return true if the successfully reconciled.
     */
    bool reconcile_with(Graph &other);

    BackEndTypes::GraphId
        get_id() const;

    BackEndTypes::NodeId
        get_origin_node_id() const;

    /** Print internal states to std output.
     * NOTE: DEBUG ONLY!
     */
    void print(const char *s) const;

private:
    friend GraphOptimizer;
    GraphImpl *m_impl;
};

class GraphOptimizerImpl;
class GraphOptimizer
{
public:
    /** default constructor.
     * @param [in] g    the graph to optimize.
     *      NOTE: will make a deep copy of the graph.
     *      The caller is responsible for data integrity.
     */
    GraphOptimizer(const Graph &g);

    /// Dtor
    ~GraphOptimizer();

    /** Run the iterative optimization.
     *      NOTE: block until convergence, or reached max iteration.
     */
    void optimize();

    /** Update the graph with optimized estimates.
     * 
     * values. No new elements will be added to @p g.
     * @param [out] g   the graph to update.
     *      NOTE: only variables known to both @p g and the optimizer
     *      will be updated. No new elements will be added. The caller
     *      is responsible for data integrity.
     */
    bool update_graph(Graph &g);
private:
    GraphOptimizerImpl *m_impl;
};

}
