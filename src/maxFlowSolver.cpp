#include "maxFlowSolver.h"

#include "correctionFlow.h"
#include "demands.h"
#include "dynamicTreeNode.h"
#include "dynamicTrees.h"
#include "embedding.h"
#include "flow.h"
#include "fractionalSolutionRounder.h"
#include "graph.h"
#include "helpers.hpp"
#include "integralFlow.h"
#include "laplacianMatrix.h"
#include "undirectedGraph.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

double MaxFlowSolver::getAbsoluteStepSize(const double primalProgress, const unsigned int numberOfEdges)
{
  double C_eps = 200;
  return (1 - primalProgress) / (33 * sqrt(C_eps * static_cast<double>(numberOfEdges)));
}

[[maybe_unused]] bool MaxFlowSolver::gammaCouplingCheck(const ResidualGraph& residualGraph, const Embedding& embedding)
{
  std::vector<double> violation(residualGraph.graph.edges.size());
  for (const auto& edge : residualGraph.graph.edges)
  {
    auto forwardCapacity = residualGraph.getForwardCapacity(edge, edge->endpoints.first);
    auto backwardCapacity = residualGraph.getBackwardCapacity(edge, edge->endpoints.first);
    double potential = 0.0;

    if (forwardCapacity != 0)
    {
      potential += 1 / forwardCapacity;
    }
    if (backwardCapacity != 0)
    {
      potential -= 1 / backwardCapacity;
    }

    double stretch = embedding.getStretch(edge);
    assert(std::min(forwardCapacity, backwardCapacity) != 0.0);

    violation[edge->id] = std::fabs(stretch - potential) * std::min(forwardCapacity, backwardCapacity);
  }

  double norm = 0;
  for (auto elem : violation)
  {
    norm += elem * elem;
  }
  norm = std::sqrt(norm);

  return norm <= 1.0 / 100.0;
}

double MaxFlowSolver::getPrimalProgress(const UndirectedGraph& undirectedGraph, const Flow& flow, double flowValue)
{
  double totalOutFlow = 0.0;
  for (auto edge : undirectedGraph.source->incident)
  {
    totalOutFlow += flow.getFlow(edge, undirectedGraph.source);
  }

  return totalOutFlow / flowValue;
}

double MaxFlowSolver::getEtaValue(const unsigned int maxCapacity, const unsigned int numberOfEdges)
{
  double importantFactor = 0;  // FIX NEEDED
  return (1.0 / 14.0) -
         ((1.0 / 7.0) * (log2(static_cast<double>(maxCapacity)) / log2(static_cast<double>(numberOfEdges)))) -
         importantFactor;
}

bool MaxFlowSolver::isEarlyTerminationPossible(double primalProgress,
                                               double flowValue,
                                               unsigned int numberOfEdges,
                                               double etaValue)
{
  return (1 - primalProgress) * flowValue <= std::pow(static_cast<double>(numberOfEdges), 0.5 - etaValue);
}

bool MaxFlowSolver::isFlowValueInfeasible(const ResidualGraph& residualGraph,
                                          const Demands& demands,
                                          const Embedding& embedding,
                                          double primalProgress)
{
  double demandedFlow = 0.0;
  for (const auto& node : residualGraph.graph.nodes)
  {
    demandedFlow += demands.getDemand(node) * embedding.getEmbedding(node);
  }

  return demandedFlow > (2.0 * static_cast<double>(residualGraph.graph.edges.size())) / (1 - primalProgress);
}

MaxFlowResult MaxFlowSolver::computeMaxFlow(const Graph& directedGraph)
{
  auto undirectedGraph = UndirectedGraph::fromDirected(directedGraph);
  undirectedGraph.addPreconditioningEdges();
  unsigned int from = 0;
  unsigned int to = undirectedGraph.getMaxCapacity() * undirectedGraph.edges.size() + 1;

  while (from < to)
  {
    auto flowValue = (from + to) / 2LU;
    auto maxFlowResult = computeMaxFlowWithPreconditioning(undirectedGraph, flowValue);
    if (maxFlowResult.isFeasible)
    {
      from = flowValue + 1;
    }
    else
    {
      to = flowValue;
    }
  }

  auto result = computeMaxFlowWithPreconditioning(undirectedGraph, from - 1);
  if (!result.isFeasible)
  {
    return result;
  }
  undirectedGraph.removePreconditioningEdges();
  getDirectedFractionalFlow(directedGraph, undirectedGraph, result.flow);
  auto integralFlow =
          FractionalSolutionRounder::roundFlow(directedGraph, result.flow, getFlowValue(directedGraph, result.flow));
  applyAugmentingPaths(directedGraph, integralFlow);

  printFlow(directedGraph, integralFlow);
  result.integralFlow = integralFlow;
  return result;
}

MaxFlowResult MaxFlowSolver::computeMaxFlowWithPreconditioning(const UndirectedGraph& undirectedGraph,
                                                               unsigned long flowValue)
{
  if (flowValue == 0)
  {
    return {true, Flow(0)};
  }

  const double etaValue = getEtaValue(undirectedGraph.getMaxCapacity(), undirectedGraph.edges.size());
  Demands demands(undirectedGraph, static_cast<double>(flowValue));
  double primalProgress = 0.0;
  Flow flow(undirectedGraph.edges.size());
  Embedding embedding(undirectedGraph.nodes.size());
  ResidualGraph residualGraph(undirectedGraph, flow);

  while ((1.0 - primalProgress) * static_cast<double>(flowValue) > 1.0)
  {
    // gammaCouplingCheck(residualGraph, embedding); DEBUG

    if (isFlowValueInfeasible(residualGraph, demands, embedding, primalProgress))
    {
      return {false, Flow(0)};
    }

    if (isEarlyTerminationPossible(
                primalProgress, static_cast<double>(flowValue), undirectedGraph.edges.size(), etaValue))
    {
      // etaValue FIX NEEDED
      // break;
    }

    // augmenting
    {
      primalProgress = getPrimalProgress(undirectedGraph, flow, static_cast<double>(flowValue));
      auto electricalFlow = ElectricalFlow(residualGraph);
      auto potentials = electricalFlow.computePotentials(undirectedGraph, demands);

      double stepSize = getAbsoluteStepSize(primalProgress, undirectedGraph.edges.size());
      flow.update(undirectedGraph, stepSize, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, stepSize, potentials);
    }
    // fixing
    {
      auto correctionFlow = CorrectionFlow(residualGraph, embedding);
      flow.applyCorrectionFlow(undirectedGraph, correctionFlow);

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto fixingDemands = Demands(undirectedGraph, correctionFlow);
      auto potentials = electricalFlow.computePotentials(undirectedGraph, fixingDemands);

      flow.update(undirectedGraph, 1.0, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, 1.0, potentials);
    }
  }
  return {true, flow};
}

bool MaxFlowSolver::containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow)
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

bool hasOutgoingFlow(const UndirectedNode* const undirectedNode,
                     const Flow& flow,
                     const std::vector<bool>& deletedEdges)
{
  return std::any_of(undirectedNode->incident.begin(),
                     undirectedNode->incident.end(),
                     [&](const auto& edge)
                     { return !deletedEdges[edge->id] && flow.getFlow(edge, undirectedNode) > 0.0; });
}

UndirectedEdge* getOutgoingFlowEdge(const UndirectedNode* const undirectedNode,
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

void MaxFlowSolver::removeFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow)
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

unsigned long MaxFlowSolver::getFlowValue(const Graph& directedGraph, const Flow& flow)
{
  double flowValue = 0.0;
  for (auto edge : directedGraph.s->outgoingEdges)
  {
    flowValue += flow.getFlow(edge);
  }
  for (auto edge : directedGraph.s->incomingEdges)
  {
    flowValue -= flow.getFlow(edge);
  }
  std::cout << "getFlowValue: " << static_cast<unsigned long>(flowValue) << std::endl;
  return static_cast<unsigned long>(flowValue);
}

void MaxFlowSolver::applyAugmentingPaths(const Graph& directedGraph, IntegralFlow& flow)
{
  while (findAugmentingPath(directedGraph, flow))
    ;
}

bool MaxFlowSolver::findAugmentingPath(const Graph& directedGraph, IntegralFlow& flow)
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

  if (incomingEdge[directedGraph.t->label])
  {
    auto additionalFlow = std::numeric_limits<unsigned long>::max();
    for (auto edge = incomingEdge[directedGraph.t->label]; edge != nullptr; edge = incomingEdge[edge->from->label])
    {
      additionalFlow = min(additionalFlow, edge->capacity - static_cast<unsigned long>(flow.getFlow(edge)));
    }
    for (auto edge = incomingEdge[directedGraph.t->label]; edge != nullptr; edge = incomingEdge[edge->from->label])
    {
      flow.updateFlow(edge, additionalFlow);
    }
    return true;
  }
  return false;
}

void MaxFlowSolver::getDirectedFractionalFlow(const Graph& directedGraph,
                                              const UndirectedGraph& undirectedGraph,
                                              Flow& flow)
{
  for (const auto& edge : directedGraph.edges)
  {
    assert(edge->undirectedEquivalent);

    auto targetEdge = undirectedGraph.targetEdges[edge->from->label];
    flow.updateFlow(targetEdge, undirectedGraph.target, edge->capacity);

    flow.updateFlow(edge->undirectedEquivalent, edge->from->undirectedEquivalent, edge->capacity);

    auto sourceEdge = undirectedGraph.sourceEdges[edge->to->label];
    flow.updateFlow(sourceEdge, edge->to->undirectedEquivalent, edge->capacity);
  }

  if (containsFlowCycles(directedGraph, undirectedGraph, flow))
  {
    removeFlowCycles(directedGraph, undirectedGraph, flow);
  }
  flow.scaleDown();

  Flow newFlow(directedGraph.edges.size());
  for (const auto& edge : directedGraph.edges)
  {
    newFlow.setFlow(edge.get(), flow.getFlow(edge->undirectedEquivalent, edge->undirectedEquivalent->endpoints.first));
  }
  flow = newFlow;
}
