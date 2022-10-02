#include "flowCycleDetector.h"

#include "dynamicTreeNode.h"
#include "dynamicTrees.h"
#include "flow.h"
#include "graph.h"
#include "undirectedEdge.h"
#include "undirectedGraph.h"
#include "undirectedNode.h"

#include <cmath>

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

bool FlowCycleDetector::containsFlowCycles(const UndirectedGraph& undirectedGraph, Flow& flow)
{
  return std::any_of(undirectedGraph.source->incident.begin(),
                     undirectedGraph.source->incident.end(),
                     [flow](const auto& edge) {
                       return edge->directedEquivalent == nullptr &&
                              !almost_equal(flow.getFlow(edge, edge->endpoints.first), 0.0, 10);
                     }) ||
         std::any_of(undirectedGraph.target->incident.begin(),
                     undirectedGraph.target->incident.end(),
                     [&](const auto& edge) {
                       return edge->directedEquivalent == nullptr &&
                              !almost_equal(flow.getFlow(edge, edge->endpoints.first), 0.0, 10);
                     });
}

bool FlowCycleDetector::hasOutgoingFlow(const UndirectedNode* const undirectedNode,
                                        const Flow& flow,
                                        const std::vector<bool>& deletedEdges)
{
  return std::any_of(undirectedNode->incident.begin(),
                     undirectedNode->incident.end(),
                     [&](const auto& edge)
                     { return !deletedEdges[edge->id] && flow.getFlow(edge, undirectedNode) > 0.0; });
}

UndirectedEdge* FlowCycleDetector::getOutgoingFlowEdge(const UndirectedNode* const undirectedNode,
                                                       const Flow& flow,
                                                       const std::vector<bool>& deletedEdges)
{
  for (const auto& edge : undirectedNode->incident)
  {
    if (deletedEdges[edge->id])
    {
      continue;
    }
    if (flow.getFlow(edge, undirectedNode) > 0.0)
    {
      return edge;
    }
  }

  return nullptr;
}

void FlowCycleDetector::removeFlowCycles(const UndirectedGraph& undirectedGraph, Flow& flow)
{
  DynamicTrees dynamicTrees(undirectedGraph.nodes);
  std::vector<bool> deletedEdges(undirectedGraph.edges.size());

  auto sourceTreeNode = dynamicTrees.dynamicTreesNodes[undirectedGraph.source->label].get();
  while (true)
  {
    auto currentNode = dynamicTrees.getRoot(sourceTreeNode);
    if (!hasOutgoingFlow(currentNode->undirectedNode, flow, deletedEdges))
    {
      // step 2 - all paths from v to t are acyclic
      if (currentNode == sourceTreeNode)
      {
        for (const auto& treeNode : dynamicTrees.dynamicTreesNodes)
        {
          if (treeNode.get() != dynamicTrees.getRoot(treeNode.get()))
          {
            flow.setFlow(treeNode->undirectedEdge, treeNode->undirectedNode, dynamicTrees.getCost(treeNode.get()));
          }
        }
        return;
      }
      for (auto edge : currentNode->undirectedNode->incident)
      {
        if (flow.getFlow(edge, currentNode->undirectedNode) < 0.0)
        {
          deletedEdges[edge->id] = true;
          auto secondEndpoint = (edge->endpoints.first == currentNode->undirectedNode) ? edge->endpoints.second
                                                                                       : edge->endpoints.first;
          auto secondEndpointTree = dynamicTrees.dynamicTreesNodes[secondEndpoint->label].get();
          if (dynamicTrees.getRoot(secondEndpointTree) != secondEndpointTree &&
              dynamicTrees.getParent(secondEndpointTree) == currentNode)
          {
            flow.setFlow(secondEndpointTree->undirectedEdge, secondEndpoint, dynamicTrees.getCost(secondEndpointTree));
            dynamicTrees.cut(secondEndpointTree);
          }
        }
      }
    }
    else  // step 1
    {
      auto edge = getOutgoingFlowEdge(currentNode->undirectedNode, flow, deletedEdges);
      auto nextNode = dynamicTrees
                              .dynamicTreesNodes[((edge->endpoints.first == currentNode->undirectedNode)
                                                          ? edge->endpoints.second
                                                          : edge->endpoints.first)
                                                         ->label]
                              .get();
      if (dynamicTrees.getRoot(nextNode) == currentNode)
      {
        // step 3 - a cycle of positive flow has been found
        auto minFlowNode = dynamicTrees.getMinCostNode(nextNode);
        auto minFlowValue =
                std::min(flow.getFlow(edge, currentNode->undirectedNode), dynamicTrees.getCost(minFlowNode));
        flow.updateFlow(edge, currentNode->undirectedNode, -minFlowValue);
        dynamicTrees.updatePath(nextNode, -minFlowValue);

        // step 4 - delete tree edges with no remaining, flow
        minFlowNode = dynamicTrees.getMinCostNode(nextNode);
        while (almost_equal(dynamicTrees.getCost(minFlowNode), 0.0, 10))
        {
          flow.setFlow(minFlowNode->undirectedEdge, minFlowNode->undirectedNode, 0.0);
          deletedEdges[minFlowNode->undirectedEdge->id] = true;
          dynamicTrees.cut(minFlowNode);
          if (dynamicTrees.getRoot(nextNode) == nextNode)
          {
            break;
          }
          minFlowNode = dynamicTrees.getMinCostNode(nextNode);
        }
      }
      else
      {
        dynamicTrees.link(currentNode, nextNode, flow.getFlow(edge, currentNode->undirectedNode));
        currentNode->undirectedEdge = edge;
      }
    }
  }
}
