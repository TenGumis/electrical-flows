#ifndef _MATCHING_GRAPH_H_
#define _MATCHING_GRAPH_H_

#include "graph.h"
#include "fancyNode.h"
#include <vector>
#include <memory>

class MatchingGraph
{
public:
    std::vector<std::shared_ptr<FancyNode>> nodesP;
    std::vector<std::shared_ptr<FancyNode>> nodesQ;

    std::vector<std::shared_ptr<MatchingEdge>> edges;
    std::vector<std::shared_ptr<MatchingEdge>> pEdges;

    static MatchingGraph toMatchingGraph(const Graph& graph, const int flow);
};

#endif // _MATCHING_GRAPH_H_