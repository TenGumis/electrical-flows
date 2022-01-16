#include "minCostFlowGraph.h"
#include <iostream>

int getDemands(MatchingEdge* matchingEdge)
{
    return std::min(matchingEdge->from->demand, matchingEdge->to->demand);
}

MinCostFlowGraph MinCostFlowGraph::toMinCostFlowGraph(const MatchingGraph& matchingGraph)
{
    MinCostFlowGraph minCostFlowGraph;

    for(int i = 0; i < matchingGraph.nodesP.size(); i++)
    {
        auto node = matchingGraph.nodesP[i];
        auto sp = std::make_shared<FlowNode>(node.get(), true);
        sp->demand = -node->demand;
        
        node->correspondingFlowNode = sp.get();

        minCostFlowGraph.nodes.push_back(sp);
    }

    for(int i = 0; i < matchingGraph.nodesQ.size(); i++)
    {
        auto node = matchingGraph.nodesQ[i];
        auto tq = std::make_shared<FlowNode>(node.get(), false);
        
        node->correspondingFlowNode = tq.get();

        minCostFlowGraph.nodes.push_back(tq);
    }

    for(int i = 0; i < matchingGraph.pEdges.size(); i++) // to powinno być po edgach z jednej strony
    {
        auto edge = matchingGraph.pEdges[i];
        int demands = getDemands(edge.get());
        for(int j = 0; j < demands; j++)
        {
            auto newFlowEdge = std::make_shared<FlowEdge>(edge->from->correspondingFlowNode, edge->to->correspondingFlowNode);

            minCostFlowGraph.edges.push_back(newFlowEdge);
            // zrobic cos z dwoma kierunkami
        }
    }

    auto v = std::make_shared<FlowNode>();
    for(int i = 0; i < minCostFlowGraph.nodes.size(); i++)
    {
        auto node = minCostFlowGraph.nodes[i];
        auto newFlowEdge1 = std::make_shared<FlowEdge>(v.get(), node.get());
        auto newFlowEdge2 = std::make_shared<FlowEdge>(node.get(), v.get());

        minCostFlowGraph.edges.push_back(newFlowEdge1);
        minCostFlowGraph.edges.push_back(newFlowEdge2);
    }
    minCostFlowGraph.nodes.push_back(v);


    //dodać jakeiś numerki do nodow


    return minCostFlowGraph;
}