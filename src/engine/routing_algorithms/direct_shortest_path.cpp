#include "engine/routing_algorithms/direct_shortest_path.hpp"
#include "engine/routing_algorithms/routing_base.hpp"
#include "engine/routing_algorithms/routing_base_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"

namespace osrm::engine::routing_algorithms
{

/// This is a stripped down version of the general shortest path algorithm.
/// The general algorithm always computes two queries for each leg. This is only
/// necessary in case of vias, where the directions of the start node is constrained
/// by the previous route.
/// This variation is only an optimization for graphs with slow queries, for example
/// not fully contracted graphs.
template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<ch::Algorithm> &engine_working_data,
                                             const DataFacade<ch::Algorithm> &facade,
                                             const PhantomEndpointCandidates &endpoint_candidates)
{
    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade.GetNumberOfNodes());
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;
    forward_heap.Clear();
    reverse_heap.Clear();

    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    std::vector<NodeID> packed_leg;
    insertNodesInHeaps(forward_heap, reverse_heap, endpoint_candidates);

    search(engine_working_data,
           facade,
           forward_heap,
           reverse_heap,
           weight,
           packed_leg,
           {},
           endpoint_candidates);

    std::vector<NodeID> unpacked_nodes;
    std::vector<EdgeID> unpacked_edges;

    if (!packed_leg.empty())
    {
        unpacked_nodes.reserve(packed_leg.size());
        unpacked_edges.reserve(packed_leg.size());
        unpacked_nodes.push_back(packed_leg.front());
        ch::unpackPath(
            facade,
            packed_leg.begin(),
            packed_leg.end(),
            [&unpacked_nodes, &unpacked_edges](std::pair<NodeID, NodeID> &edge, const auto &edge_id)
            {
                BOOST_ASSERT(edge.first == unpacked_nodes.back());
                unpacked_nodes.push_back(edge.second);
                unpacked_edges.push_back(edge_id);
            });
    }

    return extractRoute(facade, weight, endpoint_candidates, unpacked_nodes, unpacked_edges);
}

bool isCoordinateExcluded(const Coordinate &coord,
                          const std::vector<std::pair<Coordinate, Coordinate>> &excluded_coordinates)
{
    for (const auto &range : excluded_coordinates)
    {
        const auto &top_left = range.first;
        const auto &bottom_right = range.second;

        if (coord.lon >= top_left.lon && coord.lon <= bottom_right.lon &&
            coord.lat <= top_left.lat && coord.lat >= bottom_right.lat)
        {
            return true;
        }
    }
    return false;
}

template <>
InternalRouteResult directShortestPathSearch(SearchEngineData<mld::Algorithm> &engine_working_data,
                                             const DataFacade<mld::Algorithm> &facade,
                                             const PhantomEndpointCandidates &endpoint_candidates)
{
    DataFacade<mld::Algorithm>& facade_copy = const_cast<DataFacade<mld::Algorithm>&>(facade);

    std::cout << "JO nb:" << facade_copy.GetNumberOfNodes() << std::endl;

    Coordinate top_left(FloatLongitude{1.3324703652907408}, FloatLatitude{43.60817415194244});
    Coordinate bottom_right(FloatLongitude{1.3427336542260697}, FloatLatitude{43.59950619733573});
    std::vector<std::pair<Coordinate, Coordinate>> excluded_coordinates = {
        {
            top_left, bottom_right
        }
    };
    std::vector<NodeID> nodes_to_exclude;
    //std::vector<Coordinate> nodes_coordinates_to_exclude;

    for (NodeID node_id = 0; node_id < facade_copy.GetNumberOfNodes(); ++node_id)
    {
        auto node_coordinate = facade_copy.GetCoordinateOfNode(node_id);
        if (isCoordinateExcluded(node_coordinate, excluded_coordinates))
        {
            nodes_to_exclude.push_back(node_id);
            //nodes_coordinates_to_exclude.push_back(node_coordinate);
            facade_copy.DeleteNode(node_id);
            // TODO here delete node from facade
        }
    }

    std::cout << nodes_to_exclude.size() << std::endl;
    std::cout << "JO nb:" << facade_copy.GetNumberOfNodes() << std::endl;

    /*auto should_exclude_node = [&](const NodeID node_id) {
        return std::find(nodes_to_exclude.begin(), nodes_to_exclude.end(), node_id) != nodes_to_exclude.end();
    };*/

    engine_working_data.InitializeOrClearFirstThreadLocalStorage(facade_copy.GetNumberOfNodes(),
                                                                 facade_copy.GetMaxBorderNodeID() + 1);
    auto &forward_heap = *engine_working_data.forward_heap_1;
    auto &reverse_heap = *engine_working_data.reverse_heap_1;

    insertNodesInHeaps(forward_heap, reverse_heap, endpoint_candidates);

   //std::vector<NodeID> forward_nodes, reverse_nodes;

    /*for (NodeID node_id = 0; node_id < facade.GetNumberOfNodes(); ++node_id)
    {
        auto node_coordinate = facade.GetCoordinateOfNode(node_id);
        if (isCoordinateExcluded(node_coordinate, excluded_coordinates))
        {
            nodes_to_exclude.push_back(node_id);
            nodes_coordinates_to_exclude.push_back(node_coordinate);
            forward_heap.Delete(node_id);
            reverse_heap.Delete(node_id);
        }
    }*/
    //forward_heap.GetData(forward_nodes);
    //reverse_heap.GetData(reverse_nodes);

    /*for (const auto &node : forward_nodes)
    {
        if (should_exclude_node(node))
        {
            forward_heap.Delete(node);
        }
    }

    for (const auto &node : reverse_nodes)
    {
        if (should_exclude_node(node))
        {
            reverse_heap.Delete(node);
        }
    }*/

    /*for (const auto &node : nodes_to_exclude)
    {
        auto node_data = forward_heap.GetData(node);

        std::cout << "node: " << node_data.parent << std::endl;
        if (forward_heap.WasInserted(node))
        {
            forward_heap.Delete(node);
        }
        if (reverse_heap.WasInserted(node))
        {
            reverse_heap.Delete(node);
        }
    }*/

    auto unpacked_path = mld::search(engine_working_data,
                                     facade_copy,
                                     forward_heap,
                                     reverse_heap,
                                     {},
                                     INVALID_EDGE_WEIGHT,
                                     endpoint_candidates);

    return extractRoute(facade_copy,
                        unpacked_path.weight,
                        endpoint_candidates,
                        unpacked_path.nodes,
                        unpacked_path.edges);
}

} // namespace osrm::engine::routing_algorithms
