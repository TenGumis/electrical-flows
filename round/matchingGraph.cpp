#include "node.h"
#include "edge.h"
#include "matchingGraph.h"

#include <iostream>
#include <cassert>

MatchingGraph MatchingGraph::toMatchingGraph(const std::vector<std::shared_ptr<Node>>& nodes,
                                             std::vector<std::shared_ptr<Edge>>& edges,
                                             const int s,
                                             const int t,
                                             const int flow)
{
    MatchingGraph matchingGraph;
    int idCounter = 0;
    std::cerr << "MatchingGraph::toMatchingGraph start" << std::endl;
    std::cerr << "edges processing" << std::endl;
    for(int i = 0 ; i < edges.size(); i ++)
    {
        auto matchingNodeP = std::make_shared<MatchingNode>(idCounter++, edges[i]->capacity);
        auto matchingNodeQ = std::make_shared<MatchingNode>(idCounter++, edges[i]->capacity);
        edges[i]->matchingNodeP = matchingNodeP.get();
        edges[i]->matchingNodeQ = matchingNodeQ.get();

        matchingGraph.nodesP.push_back(matchingNodeP);
        matchingGraph.nodesQ.push_back(matchingNodeQ);

        auto newEdge = std::make_shared<MatchingEdge>(matchingNodeP.get(),
                                                      matchingNodeQ.get(),
                                                      edges[i]->flow);

        matchingGraph.edges.push_back(newEdge);
        matchingNodeP->edges.push_back(newEdge);
        matchingNodeQ->edges.push_back(newEdge);
    }
    std::cerr << "nodes processing" << std::endl;
    for(int i = 0 ; i < nodes.size(); i ++)
    {
        std::cerr << i << "/" << nodes.size() << " nodeId: " << nodes[i]->id << std::endl;

        if(nodes[i]->id == s || nodes[i]->id == t)
        {
            continue;
        }

        auto matchingNodeP = std::make_shared<MatchingNode>(idCounter++);
        auto matchingNodeQ = std::make_shared<MatchingNode>(idCounter++);

        matchingGraph.nodesP.push_back(matchingNodeP);
        matchingGraph.nodesQ.push_back(matchingNodeQ);

        auto mainNewEdge = std::make_shared<MatchingEdge>(matchingNodeP.get(),
                                                          matchingNodeQ.get());

        matchingGraph.edges.push_back(mainNewEdge);
        matchingNodeP->edges.push_back(mainNewEdge);
        matchingNodeQ->edges.push_back(mainNewEdge);

        //z q wychodzace
        std::cerr << "z q wychodzace" << std::endl;
        for(int j = 0; j < nodes[i]->outgoingEdges.size(); j++)
        {
            auto outgoingEdge = nodes[i]->outgoingEdges[j];
            matchingNodeQ->demand += outgoingEdge->capacity;
            mainNewEdge->flow += outgoingEdge->flow;

            auto newEdge = std::make_shared<MatchingEdge>(outgoingEdge->matchingNodeP,
                                                          matchingNodeQ.get(),
                                                          outgoingEdge->capacity - outgoingEdge->flow);

            matchingGraph.edges.push_back(newEdge);
            matchingNodeQ->edges.push_back(newEdge);
            assert(outgoingEdge->matchingNodeP);
            outgoingEdge->matchingNodeP->edges.push_back(newEdge);
        }

        //do p wchodzace
        std::cerr << "do p wchodzace" << std::endl;
        for(int j = 0; j < nodes[i]->incomingEdges.size(); j++)
        {
            auto incomingEdge = nodes[i]->incomingEdges[j];
            matchingNodeP->demand += incomingEdge->capacity;

            auto newEdge = std::make_shared<MatchingEdge>(matchingNodeP.get(),
                                                          incomingEdge->matchingNodeQ,
                                                          incomingEdge->capacity - incomingEdge->flow);

            matchingGraph.edges.push_back(newEdge);
            matchingNodeP->edges.push_back(newEdge);
            assert(incomingEdge->matchingNodeQ);
            incomingEdge->matchingNodeQ->edges.push_back(newEdge);
        }
    }

    std::cerr << "t node processing" << std::endl;
    auto matchingNodePT = std::make_shared<MatchingNode>(idCounter++);
    matchingGraph.nodesP.push_back(matchingNodePT);
    for(int j = 0; j < nodes[t]->incomingEdges.size(); j++)
    {
        auto incomingEdge = nodes[t]->incomingEdges[j];
        matchingNodePT->demand += incomingEdge->capacity;

        auto newEdge = std::make_shared<MatchingEdge>(matchingNodePT.get(),
                                                      incomingEdge->matchingNodeQ,
                                                      incomingEdge->capacity - incomingEdge->flow);

        matchingGraph.edges.push_back(newEdge);
        matchingNodePT->edges.push_back(newEdge);
        incomingEdge->matchingNodeQ->edges.push_back(newEdge);
    }
    matchingNodePT->demand -= flow;

    std::cerr << "s node processing" << std::endl;
    auto matchingNodeQS = std::make_shared<MatchingNode>(idCounter++);
    matchingGraph.nodesQ.push_back(matchingNodeQS);
    for(int j = 0; j < nodes[s]->outgoingEdges.size(); j++)
    {
        auto outgoingEdge = nodes[s]->outgoingEdges[j];
        matchingNodeQS->demand += outgoingEdge->capacity;

        auto newEdge = std::make_shared<MatchingEdge>(outgoingEdge->matchingNodeP,
                                                       matchingNodeQS.get(),
                                                       outgoingEdge->capacity - outgoingEdge->flow);

        matchingGraph.edges.push_back(newEdge);
        matchingNodeQS->edges.push_back(newEdge);
        outgoingEdge->matchingNodeP->edges.push_back(newEdge);
    }
    matchingNodeQS->demand -= flow;

    return matchingGraph;
}

