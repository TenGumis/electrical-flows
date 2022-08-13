#ifndef _MATCHING_GRAPH_H_
#define _MATCHING_GRAPH_H_

#include "matchingNode.h"
#include "matchingEdge.h"
#include <vector>
#include <memory>

class MatchingGraph
{
public:
    std::vector<std::shared_ptr<MatchingNode>> nodesP;
    std::vector<std::shared_ptr<MatchingNode>> nodesQ;

    std::vector<std::shared_ptr<MatchingEdge>> edges;

    static MatchingGraph toMatchingGraph(const std::vector<std::shared_ptr<Node>>& nodes,
                                         std::vector<std::shared_ptr<Edge>>& edges,       
                                         const int s,
                                         const int t,
                                         const int flow);

    void toNonPerfectMatching();
    void toPerfectMatching();
};

#endif // _MATCHING_GRAPH_H_