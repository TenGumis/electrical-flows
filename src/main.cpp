#include "graph.h"
#include "maxFlowSolver.h"

#include <iostream>

Graph getInput(std::istream& inputStream)
{
  Graph graph;
  int numberOfNodes, numberOfEdges;
  inputStream >> numberOfNodes >> numberOfEdges;

  for (int i = 0; i < numberOfNodes; i++)
  {
    std::shared_ptr<Node> newNode = std::make_shared<Node>(i);
    graph.addNode(newNode);
  }

  for (int i = 0; i < numberOfEdges; i++)
  {
    int from, to, capacity;
    inputStream >> from >> to >> capacity;
    auto nodeFrom = graph.nodes[from].get();
    auto nodeTo = graph.nodes[to].get();
    auto newEdge = std::make_shared<Edge>(nodeFrom, nodeTo, capacity, i);

    graph.addEdge(newEdge);
    nodeFrom->outgoing.push_back(newEdge.get());
    nodeTo->incoming.push_back(newEdge.get());
  }

  int s, t;
  inputStream >> s >> t;
  graph.s = graph.nodes[s].get();
  graph.t = graph.nodes[t].get();

  return graph;
}

int main()
{
  int z;
  std::cin >> z;
  while (z--)
  {
    auto directedGraph = getInput(std::cin);
    double temporaryFlowValue = 120;  // TODO
    MaxFlowSolver::computeMaxFlow(directedGraph, temporaryFlowValue);
  }
}