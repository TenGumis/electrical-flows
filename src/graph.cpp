#include "graph.h"

void Graph::addNode(std::shared_ptr<Node> newNode)
{
    this->nodes.push_back(newNode);
}

void Graph::addEdge(std::shared_ptr<Edge> newEdge)
{
    this->edges.push_back(newEdge);
}

void Graph::updateFlow(const std::vector<double>& flow)
{
    for(int i = 0 ; i < edges.size(); i++)
    {
        edges[i]->forwardCapacity -= flow[i];
        edges[i]->backwardCapacity += flow[i];  
    }
}