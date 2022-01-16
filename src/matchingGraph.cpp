#include "matchingGraph.h"

MatchingGraph MatchingGraph::toMatchingGraph(const Graph& graph, const int flow)
{
    MatchingGraph matchingGraph;

    for(int i = 0 ; i < graph.edges.size(); i ++)
    {
        auto pe = std::make_shared<FancyNode>(graph.edges[i].get());
        auto qe = std::make_shared<FancyNode>(graph.edges[i].get());
        graph.edges[i]->matchingNodeP = pe.get();
        graph.edges[i]->matchingNodeQ = qe.get();

        matchingGraph.nodesP.push_back(pe);
        matchingGraph.nodesQ.push_back(qe);

        auto newEdge1 = std::make_shared<MatchingEdge>(pe.get(), qe.get());
        auto newEdge2 = std::make_shared<MatchingEdge>(qe.get(), pe.get());

        matchingGraph.edges.push_back(newEdge1);
        matchingGraph.pEdges.push_back(newEdge1);
        matchingGraph.edges.push_back(newEdge2);
    }

    for(int i = 0 ; i < graph.nodes.size(); i ++)
    {
        if(graph.nodes[i].get() == graph.s || graph.nodes[i].get() == graph.t)
        {
            continue;
        }

        auto pv = std::make_shared<FancyNode>(graph.nodes[i].get());
        auto qv = std::make_shared<FancyNode>(graph.nodes[i].get());

        matchingGraph.nodesP.push_back(pv);
        matchingGraph.nodesQ.push_back(qv);

        auto newEdge1 = std::make_shared<MatchingEdge>(pv.get(), qv.get());
        auto newEdge2 = std::make_shared<MatchingEdge>(qv.get(), pv.get());

        matchingGraph.edges.push_back(newEdge1);
        matchingGraph.pEdges.push_back(newEdge1);
        matchingGraph.edges.push_back(newEdge2);

        //z q wychodzace
        for(int j = 0; j < graph.nodes[i]->outgoing.size(); j++)
        {
            auto tmp = graph.nodes[i]->outgoing[j];
            qv->demand += tmp->capacity;

            auto newEdge1 = std::make_shared<MatchingEdge>(qv.get(), tmp->matchingNodeP);
            auto newEdge2 = std::make_shared<MatchingEdge>(tmp->matchingNodeP, qv.get());

            matchingGraph.edges.push_back(newEdge1);
            matchingGraph.edges.push_back(newEdge2);
            matchingGraph.pEdges.push_back(newEdge2);
        }

        //do p wchodzace
        for(int j = 0; j < graph.nodes[i]->incoming.size(); j++)
        {
            auto tmp = graph.nodes[i]->incoming[j];
            pv->demand += tmp->capacity;

            auto newEdge1 = std::make_shared<MatchingEdge>(pv.get(), tmp->matchingNodeQ);
            auto newEdge2 = std::make_shared<MatchingEdge>(tmp->matchingNodeQ, pv.get());

            matchingGraph.edges.push_back(newEdge1);
            matchingGraph.pEdges.push_back(newEdge1);
            matchingGraph.edges.push_back(newEdge2);
        }
    }

    auto pt = std::make_shared<FancyNode>(graph.t);
    for(int j = 0; j < graph.t->incoming.size(); j++)
    {
        auto tmp = graph.t->incoming[j];
        pt->demand += tmp->capacity;

        auto newEdge1 = std::make_shared<MatchingEdge>(pt.get(), tmp->matchingNodeQ);
        auto newEdge2 = std::make_shared<MatchingEdge>(tmp->matchingNodeQ, pt.get());

        matchingGraph.edges.push_back(newEdge1);
        matchingGraph.pEdges.push_back(newEdge1);
        matchingGraph.edges.push_back(newEdge2);
        
    }
    pt->demand -= flow;

    auto qs = std::make_shared<FancyNode>(graph.s);
    for(int j = 0; j < graph.s->outgoing.size(); j++)
    {
        auto tmp = graph.s->outgoing[j];
        qs->demand += tmp->capacity;

        auto newEdge1 = std::make_shared<MatchingEdge>(qs.get(), tmp->matchingNodeP);
        auto newEdge2 = std::make_shared<MatchingEdge>(tmp->matchingNodeP, qs.get());

        matchingGraph.edges.push_back(newEdge1);
        matchingGraph.edges.push_back(newEdge2);
        matchingGraph.pEdges.push_back(newEdge2);
    }
    qs->demand -= flow;

    matchingGraph.nodesP.push_back(pt);
    matchingGraph.nodesQ.push_back(qs);
    
    return matchingGraph;
}

