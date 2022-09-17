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
    nodeFrom->outgoingEdges.push_back(newEdge.get());
    nodeTo->incomingEdges.push_back(newEdge.get());
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
    unsigned long temporaryFlowValue = 125;  // TODO
    auto result = MaxFlowSolver::computeMaxFlow(directedGraph, temporaryFlowValue);
    if (result.isFeasible)
    {
      double totalFlow = 0.0;
      for (const auto edge : directedGraph.s->outgoingEdges)
      {
        totalFlow += result.flow.getFlow(edge);
      }
      std::cout << "total flow: " << totalFlow << std::endl;
      for (const auto& edge : directedGraph.edges)
      {
        std::cout << edge->id << ": " << result.flow.getFlow(edge.get()) << std::endl;
      }
    }
    else
    {
      std::cout << "not feasible flow\n";
    }
  }
}