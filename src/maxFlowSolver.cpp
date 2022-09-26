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
#include "violation.h"

#include <cassert>
#include <cmath>
#include <iomanip>
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

double geAbsoluteStepSize(const double primalProgress, const double m)
{
  double C_eps = 200;

  return (1 - primalProgress) / (33 * sqrt(C_eps * m));
}

bool gammaCouplingCheck(const ResidualGraph& residualGraph, const Embedding& embedding, Violation& violation)
{
  std::cerr << "gammaCouplingCheck" << std::endl;
  for (const auto& edge : residualGraph.graph.edges)
  {
    auto forwardCapacity = residualGraph.getForwardCapacity(edge, edge->endpoints.first);
    auto backwardCapacity = residualGraph.getBackwardCapacity(edge, edge->endpoints.first);
    double potential = 0.0;

    if (forwardCapacity != 0)  // TODO dividing by 0
    {
      potential += 1 / forwardCapacity;
    }
    if (backwardCapacity != 0)  // TODO dividing by 0
    {
      potential -= 1 / backwardCapacity;
    }

    double stretch = embedding.getStretch(edge);
    std::cerr << forwardCapacity << " " << backwardCapacity << " " << stretch << " " << potential << std::endl;
    assert(std::min(forwardCapacity, backwardCapacity) != 0.0);

    violation.violation[edge->id] = std::fabs(stretch - potential) * std::min(forwardCapacity, backwardCapacity);
  }

  double norm = 0;
  for (auto elem : violation.violation)
  {
    norm += elem * elem;
  }
  norm = std::sqrt(norm);

  std::streamsize ss = std::cerr.precision();
  std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 2) << norm << "\n";
  std::cerr << std::setprecision(ss);

  assert(norm <= 1.0 / 100.);

  return true;
}

void capacityConstraintCheck(const UndirectedGraph& graph, const Flow& flow)
{
  for (const auto& edge : graph.edges)
  {
    assert(flow.getFlow(edge.get(), edge->endpoints.first) <= edge->capacity);
    assert(flow.getFlow(edge.get(), edge->endpoints.first) >= -edge->capacity);
  }
  // -ue- <= fe <= ue+;
}

void flowConservationConstraintsCheck(const UndirectedGraph& graph, const Flow& flow, const Demands& demands)
{
  for (auto node : graph.nodes)
  {
    /*
    double enteringFlow = 0.0;
    for (auto edge : node->incoming)
    {
      enteringFlow += flow.v[edge->id];
    }
    double leavingFlow = 0.0;
    for (auto edge : node->outgoing)
    {
      leavingFlow += flow.v[edge->id];
    }
    */
    // assert(almost_equal(enteringFlow - leavingFlow, demands.getDemand(node), 10));
  }
}

void feasibilityCheck(const UndirectedGraph& undirectedGraph, const Flow& flow, const Demands& demands)
{
  // flowConservationConstraintsCheck(graph, flow, demands);
  capacityConstraintCheck(undirectedGraph, flow);
}

double getPrimalProgress(const UndirectedGraph& undirectedGraph, const Flow& flow, double flowValue)
{
  double totalOutFlow = 0.0;
  for (auto edge : undirectedGraph.source->incident)
  {
    totalOutFlow += flow.getFlow(edge, undirectedGraph.source);
  }

  return totalOutFlow / flowValue;
}

double getEtaValue(const double maxCapacity, const double numberOfEdges)
{
  double importantFactor = 0;  // TODO change to proper computations
  return (1.0 / 14.0) - ((1.0 / 7.0) * (log2(maxCapacity) / log2(numberOfEdges))) - importantFactor;
}

bool isEarlyTerminationPossible(double primalProgress, double flowValue, unsigned int numberOfEdges, double etaValue)
{
  return (1 - primalProgress) * flowValue <= std::pow(static_cast<double>(numberOfEdges), 0.5 - etaValue);
}

bool isFlowValueInfeasible(const ResidualGraph& residualGraph,
                           const Demands& demands,
                           const Embedding& embedding,
                           double primalProgress)
{
  double demandedFlow = 0.0;
  for (const auto& node : residualGraph.graph.nodes)
  {
    demandedFlow += demands.getDemand(node) * embedding.getEmbedding(node);
  }

  return demandedFlow > (2.0 * residualGraph.graph.edges.size()) / (1 - primalProgress);
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
  // assert(undirectedGraph.edges.size() == 6);
  std::cerr << "LOL" << std::endl;
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
  std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 2);
  int sum = 0;
  for (auto edge : undirectedGraph.edges)
  {
    sum += edge->capacity;
    std::cerr << edge->endpoints.first->label << " " << edge->endpoints.second->label << " ";
    std::cerr << edge->capacity << std::endl;
  }
  std::cerr << sum << std::endl;
  // assert(sum == 141);

  const double etaValue = getEtaValue(undirectedGraph.getMaxCapacity(), undirectedGraph.edges.size());
  Demands demands(undirectedGraph, flowValue);
  std::cerr << "demands: " << std::endl;
  for (auto node : undirectedGraph.nodes)
  {
    std::cerr << demands.getDemand(node) << " ";
  }
  std::cerr << std::endl;
  double primalProgress = 0.0;
  double dualProgress = 0.0;
  Flow flow(undirectedGraph.edges.size());
  Embedding embedding(undirectedGraph.nodes.size());
  Violation violation(undirectedGraph.edges.size());
  ResidualGraph residualGraph(undirectedGraph, flow);

  // int community-code = 20000;  // flowValue/stepSize;
  int steps = 0;
  while ((1 - primalProgress) * flowValue > 1)  //! almost_equal(primalProgress, 1.0, 10) && primalProgress < 1.0)
  {
    std::cerr << "STEP: " << steps++ << std::endl;
    if (steps > 40000) break;
    gammaCouplingCheck(residualGraph, embedding, violation);

    if (isFlowValueInfeasible(residualGraph, demands, embedding, primalProgress))
    {
      std::cerr << "to much flow1111 " << flowValue << std::endl;
      return {false, Flow(0)};
    }

    if (isEarlyTerminationPossible(primalProgress, flowValue, undirectedGraph.edges.size(), etaValue))
    {
      std::cout << "isEarlyTerminationPossible" << std::endl;
      /*
      That is, once we know that we are within an additive factor of m 2 −η of the target ﬂow, we do not
      execute progress steps anymore. Observe that in that case we can just round our current ﬂow and
      1
      compute the optimal solution using at most m 2 −η augmenting paths computation (see, e.g., [43] for
      details)
      */

      // round()
      // performAugmentingPathsComputation() m^(1/2 - etaValue) razy
      // break;
    }
    // augumenting
    {
      // std::cerr << "-------------------------------------------------------" << std::endl;

      primalProgress = getPrimalProgress(undirectedGraph, flow, flowValue);
      // dualProgress = sigma * y;
      // std::cerr << "primalProgress: " << primalProgress << std::endl;
      // std::cerr << "dualProgress: " << dualProgress << std::endl;
      // std::cerr << "steps: " << steps << std::endl;

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto potentials = electricalFlow.computePotentials(undirectedGraph, demands);
      // printInducedFlow(undirectedGraph, potentials, electricalFlow.resistances);
      /*
       std::cerr << "stepSize: " << geAbsoluteStepSize(primalProgress, undirectedGraph.edges.size()) << std::endl;
       std::cerr << "potentials: " << std::endl;
       for (auto elem : potentials)
       {
         std::cerr << elem << " ";
       }
       std::cerr << std::endl;
       */

      double stepSize = geAbsoluteStepSize(primalProgress, undirectedGraph.edges.size());
      flow.update(undirectedGraph, stepSize, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, stepSize, potentials);
      // printFlow(undirectedGraph, flow);
      // printEmbedding(undirectedGraph, embedding);

      // std::cerr << "primalProgress: " << getPrimalProgress(undirectedGraph, flow, flowValue) << std::endl;
    }

    // fixing
    {
      // gammaCouplingCheck(graph, embedding, );
      auto correctionFlow = CorrectionFlow(residualGraph, embedding);
      // printCorrections(undirectedGraph, correctionFlow);

      flow.applyCorrectionFlow(undirectedGraph, correctionFlow);
      // printFlow(undirectedGraph, flow);

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto fixingDemands = Demands(undirectedGraph, correctionFlow);
      /*
      std::cerr << "fixingDemands: " << std::endl;
      for (auto node : undirectedGraph.nodes)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << fixingDemands.getDemand(node)
                  << " ";
        std::cerr << std::setprecision(ss);
      }
       */
      auto potentials = electricalFlow.computePotentials(undirectedGraph, fixingDemands);
      // printInducedFlow(undirectedGraph, potentials, electricalFlow.resistances);

      flow.update(undirectedGraph, 1.0, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, 1.0, potentials);

      // printFlow(undirectedGraph, flow);
    }
    feasibilityCheck(undirectedGraph, flow, demands);
  }
  return {true, flow};
}

bool MaxFlowSolver::containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow)
{
  for (const auto& edge : undirectedGraph.source->incident)
  {
    if (edge->directedEquivalent == nullptr && !almost_equal(flow.getFlow(edge, edge->endpoints.first), 0.0, 10))
    {
      return true;
    }
  }
  for (const auto& edge : undirectedGraph.target->incident)
  {
    if (edge->directedEquivalent == nullptr && !almost_equal(flow.getFlow(edge, edge->endpoints.first), 0.0, 10))
    {
      return true;
    }
  }
  return false;
}

bool hasOutgoingFlow(const UndirectedNode* const undirectedNode,
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
      return true;
    }
  }

  return false;
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
      // TODO usun krawedzie do currentNode

      // WAZNE pomysl na usuwanie jest taki, ze zmienimy z ostatnia dobrą krawedzia na liscie
      // pytanie czy to nie zepsuje pozniej ID krawedzi
      // chyba nie bo zmieniamy tylko na liscie incydentnych w wierzcholkach
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

        std::cerr << dynamicTrees.getCost(dynamicTrees.dynamicTreesNodes[0].get()) << std::endl;
        std::cerr << dynamicTrees.getCost(dynamicTrees.dynamicTreesNodes[1].get()) << std::endl;
        std::cerr << dynamicTrees.getCost(dynamicTrees.dynamicTreesNodes[2].get()) << std::endl;

        // step 4 - delete tree edges with no remaining, flow
        minFlowNode = dynamicTrees.getMinCostNode(nextNode);
        while (almost_equal(dynamicTrees.getCost(minFlowNode), 0.0, 10))
        {
          flow.setFlow(minFlowNode->undirectedEdge, minFlowNode->undirectedNode, 0.0);
          deletedEdges[minFlowNode->undirectedEdge->id] = true;
          // TODO
          // usun minFlowNode, paren(minFlowNode) z grafu
          // mozna zrobić lazy tj. wpisac 0.0 na sztywno i wyjdzie ze jej nie ma w poszukiwaniach
          dynamicTrees.cut(minFlowNode);
          minFlowNode = dynamicTrees.getMinCostNode(nextNode);
        }
      }
      else
      {
        dynamicTrees.link(currentNode, nextNode, flow.getFlow(edge, currentNode->undirectedNode));
        currentNode->undirectedEdge = edge;  // dodac do linka equivalent edge;
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

  printFlow(undirectedGraph, flow);
  std::cerr << "\nbefore containsFlowCycles check" << std::endl;
  if (containsFlowCycles(directedGraph, undirectedGraph, flow))
  {
    std::cerr << "\nremoving Flow Cycles" << std::endl;
    removeFlowCycles(directedGraph, undirectedGraph, flow);
  }
  std::cerr << "\nafter containsFlowCycles check" << std::endl;

  printFlow(undirectedGraph, flow);
  std::cerr << "\npodziel przez 2" << std::endl;
  flow.scaleDown();
  printFlow(undirectedGraph, flow);

  Flow newFlow(directedGraph.edges.size());

  for (const auto& edge : directedGraph.edges)
  {
    newFlow.setFlow(edge.get(), flow.getFlow(edge->undirectedEquivalent, edge->undirectedEquivalent->endpoints.first));
  }

  flow = newFlow;
  printFlow(directedGraph, flow);
}
