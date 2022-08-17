#include "node.h"
#include "edge.h"
#include "matchingGraph.h"
#include <iostream>
#include <vector>
#include <memory>
#include <cassert>

void printGraph(const MatchingGraph& matchingGraph)
{
    std::cout << "nodesP: " << matchingGraph.nodesP.size() << std::endl;
    double totalFlow = 0;
    for(auto node : matchingGraph.nodesP)
    {
        double flow = 0;
        for(auto edge: node->edges)
        {
            flow += edge->flow;
            totalFlow += edge->flow;
            assert(edge->pNode->id == node->id);
        }
        std::cout << node->id <<" "<< flow << "/" << node->demand << " " << node->edges.size() << std::endl; 
    }
    std::cout << "totalFlow: " << totalFlow << std::endl;

    std::cout << "nodesQ: " << matchingGraph.nodesQ.size() << std::endl;
    totalFlow = 0;
    for(auto node : matchingGraph.nodesQ)
    {
        double flow = 0;
        for(auto edge: node->edges)
        {
            flow += edge->flow;
            totalFlow += edge->flow;
            assert(edge->qNode->id == node->id);
        }
        std::cout << node->id <<" "<< flow << "/" << node->demand << " " << node->edges.size() << std::endl; 
    }
    std::cout << "totalFlow: " << totalFlow << std::endl;

    std::cout << "edges: " << matchingGraph.edges.size() << std::endl;
    for(auto edge : matchingGraph.edges)
    {
        std::cout << edge->pNode->id <<" "<< edge->qNode->id << " " << edge->flow << std::endl; 
    }
    
}

void solve()
{
    int n,m;
    std::cin >> n >> m;
    std::vector<std::shared_ptr<Node>> nodes(n);
    std::vector<std::shared_ptr<Edge>> edges;

    for(int i = 0; i < n; i++)
    {
        nodes[i] = std::make_shared<Node>();
        nodes[i]->id = i;
    }
    std::cerr << "Processing input" << std::endl;
    for(int i = 0; i < m; i++)
    {
        std::cerr << i << "/" << m << std::endl;
        int a,b,capacity;
        double flow;
        std::cin >> a >> b >> capacity >> flow;
        auto edge = std::make_shared<Edge>(nodes[a].get(), nodes[b].get(), capacity, flow);
        nodes[a]->outgoingEdges.push_back(edge.get());
        nodes[b]->incomingEdges.push_back(edge.get());
        edges.push_back(edge);
    }
    int s, t, flowValue;
    std::cin >> s >> t >> flowValue;
    std::cerr << "Default graph created" << std::endl;
    
    auto matchingGraph = MatchingGraph::toMatchingGraph(nodes, edges, s, t, flowValue);
    std::cerr << "fractional b-matching graph created" << std::endl;
    printGraph(matchingGraph);

    matchingGraph.toNonPerfectMatching();
    std::cerr << "non-perfect matching graph created" << std::endl;
    printGraph(matchingGraph);

    matchingGraph.toPerfectMatching();
    std::cerr << "perfect matching graph created" << std::endl;
    //printGraph(matchingGraph);
   
}

int main()
{
    int z;
    std::cin >>z;
    while(z--)
    {
        solve();
    }
}