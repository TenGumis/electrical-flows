#ifndef _MIN_COST_FLOW_GRAPH_H_
#define _MIN_COST_FLOW_GRAPH_H_

#include "graph.h"
#include "matchingGraph.h"
#include "flowNode.h"
#include "flowEdge.h"
#include <vector>
#include <memory>

class FlowNode;
class FlowEdge;

class MinCostFlowGraph
{
public:
    
    std::vector<std::shared_ptr<FlowNode>> nodes;
    std::vector<std::shared_ptr<FlowEdge>> edges;

    static MinCostFlowGraph toMinCostFlowGraph(const MatchingGraph& graph);
};

#endif // _MIN_COST_FLOW_GRAPH_H_