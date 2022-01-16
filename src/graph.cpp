#include "graph.h"

void Graph::addNode(std::shared_ptr<Node> newNode)
{
    this->nodes.push_back(newNode);
}

void Graph::addEdge(std::shared_ptr<Edge> newEdge)
{
    this->edges.push_back(newEdge);
}