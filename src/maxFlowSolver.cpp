#include "maxFlowSolver.h"

#include "augmentingPathFinder.h"
#include "correctionFlow.h"
#include "demands.h"
#include "embedding.h"
#include "flow.h"
#include "flowCycleDetector.h"
#include "fractionalSolutionRounder.h"
#include "graph.h"
#include "laplacianMatrix.h"
#include "undirectedGraph.h"

#include <cassert>
#include <cmath>
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
  double C_eps = 200.0;
  return (1.0 - primalProgress) / (33.0 * sqrt(C_eps * static_cast<double>(numberOfEdges)));
}

double MaxFlowSolver::l4norm(const std::vector<double>& vector)
{
  double norm = 0;
  for (auto elem : vector)
  {
    norm += elem * elem * elem * elem;
  }
  return std::pow(norm, 1.0 / 4.0);
}

double MaxFlowSolver::getStepSize(const ResidualGraph& residualGraph,
                                  const std::vector<double>& potentials,
                                  const std::vector<double>& resistances)
{
  std::vector<double> congestion(residualGraph.graph.edges.size());

  for (unsigned int i = 0; i < residualGraph.graph.edges.size(); i++)
  {
    auto edge = residualGraph.graph.edges[i];
    double inducedFlow = (potentials[edge->endpoints.second->label] - potentials[edge->endpoints.first->label]) /
                         resistances[edge->id];
    congestion[i] = inducedFlow / residualGraph.getSymmetricalResidualCapacity(edge);
  }

  return 1.0 / (33.0 * l4norm(congestion));
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
  AugmentingPathFinder::applyAugmentingPaths(directedGraph, integralFlow);
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
    // gammaCouplingCheck(residualGraph, embedding); // DEBUG

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

      auto stepSize = getStepSize(residualGraph, potentials, electricalFlow.resistances);
      assert(stepSize >= getAbsoluteStepSize(primalProgress, undirectedGraph.edges.size()));
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
  return static_cast<unsigned long>(flowValue);
}

void MaxFlowSolver::getDirectedFractionalFlow(const Graph& directedGraph,
                                              const UndirectedGraph& undirectedGraph,
                                              Flow& flow)
{
  for (const auto& directedEdge : directedGraph.edges)
  {
    assert(directedEdge->undirectedEquivalent);
    auto undirectedEdge = directedEdge->undirectedEquivalent;

    auto targetEdge = undirectedGraph.targetEdges[undirectedEdge->endpoints.first->label];
    flow.updateFlow(targetEdge, undirectedGraph.target, undirectedEdge->capacity);

    flow.updateFlow(undirectedEdge, undirectedEdge->endpoints.first, undirectedEdge->capacity);

    auto sourceEdge = undirectedGraph.sourceEdges[undirectedEdge->endpoints.second->label];
    flow.updateFlow(sourceEdge, undirectedEdge->endpoints.second, undirectedEdge->capacity);
  }

  flow.scaleDown();
  if (FlowCycleDetector::containsFlowCycles(undirectedGraph, flow))
  {
    FlowCycleDetector::removeFlowCycles(undirectedGraph, flow);
  }

  Flow newFlow(directedGraph.edges.size());
  for (const auto& edge : directedGraph.edges)
  {
    newFlow.setFlow(edge.get(), flow.getFlow(edge->undirectedEquivalent, edge->undirectedEquivalent->endpoints.first));
  }
  flow = newFlow;
}
