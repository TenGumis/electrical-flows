#include "augmentingPathFinder.h"

#include <limits>
#include <queue>

void AugmentingPathFinder::applyAugmentingPaths(const Graph& directedGraph, IntegralFlow& flow)
{
  while (true)
  {
    std::queue<const Node*> nodesQueue;
    std::vector<Edge*> incomingEdge(directedGraph.nodes.size());
    nodesQueue.push(directedGraph.s);
    while (!nodesQueue.empty())
    {
      auto currentNode = nodesQueue.front();
      nodesQueue.pop();
      for (auto edge : currentNode->outgoingEdges)
      {
        if (incomingEdge[edge->to->label] == nullptr && edge->to != directedGraph.s &&
            edge->capacity > static_cast<int>(flow.getFlow(edge)))
        {
          incomingEdge[edge->to->label] = edge;
          nodesQueue.push(edge->to);
        }
      }
    }

    if (!incomingEdge[directedGraph.t->label])
    {
      return;
    }
    auto additionalFlow = std::numeric_limits<unsigned long>::max();
    for (auto edge = incomingEdge[directedGraph.t->label]; edge != nullptr; edge = incomingEdge[edge->from->label])
    {
      additionalFlow = std::min(additionalFlow, edge->capacity - static_cast<unsigned long>(flow.getFlow(edge)));
    }
    for (auto edge = incomingEdge[directedGraph.t->label]; edge != nullptr; edge = incomingEdge[edge->from->label])
    {
      flow.updateFlow(edge, additionalFlow);
    }
  }
}
