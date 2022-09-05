#include "maxFlowSolver.h"

#include "correctionFlow.h"
#include "demands.h"
#include "embedding.h"
#include "flow.h"
#include "graph.h"
#include "helpers.hpp"
#include "laplacianMatrix.h"
#include "undirectedGraph.h"
#include "violation.h"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

std::vector<double> getCongestionVector(const ResidualGraph& residualGraph,
                                        const std::vector<double>& potentials,
                                        const std::vector<double>& resistances)
{
  std::vector<double> congestionVector(residualGraph.graph.edges.size());
  for (const auto& edge : residualGraph.graph.edges)
  {
    double inducedFlow = (potentials[edge->endpoints.second->label] - potentials[edge->endpoints.first->label]) /
                         resistances[edge->id];
    congestionVector[edge->id] = inducedFlow / residualGraph.getSymmetricalResidualCapacity(edge);
  }

  return congestionVector;
}

double geAbsoluteStepSize(const double primalProgress, const double m)
{
  double C_eps = 200;

  return (1 - primalProgress) / (33 * sqrt(C_eps * m));
}

double getRelativeProgressStep(const double m)
{
  double C_eps = 200;

  return 1 / (33 * sqrt(C_eps * m));
}

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
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

    /*
    std::streamsize ss = std::cerr.precision();
    std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 2) << std::fabs(stretch -
    potential) << "\n"; std::cerr << std::fabs(stretch - potential) << std::endl; std::cerr <<
    violation.getViolation(edge) << std::endl; std::cerr << violation.getViolation(edge) / std::min(forwardCapacity,
    backwardCapacity) << std::endl; std::cerr << std::setprecision(ss);

    auto tmp = std::fabs(stretch - potential);
    if(almost_equal(stretch, potential, 100))
    {
        tmp = 0.0;
    }
    if (tmp > violation.getViolation(edge) / std::min(forwardCapacity, backwardCapacity))
    {
      assert(false);
      return false;
    }
    */
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

  return demandedFlow > (2.0 * residualGraph.graph.nodes.size()) / (1 - primalProgress);
}

bool MaxFlowSolver::computeMaxFlow(const UndirectedGraph& undirectedGraph)
{
  // TODO
}

bool MaxFlowSolver::computeMaxFlow(const UndirectedGraph& undirectedGraph, double flowValue)
{
  std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 2);
  int sum = 0;
  for (auto edge : undirectedGraph.edges)
  {
    sum += edge->capacity;
    std::cerr << edge->endpoints.first->label << " " << edge->endpoints.second->label << " ";
    std::cerr << edge->capacity << std::endl;
  }
  std::cerr << sum << std::endl;
  assert(sum == 141);

  const double etaValue = getEtaValue(0, 0);  // TODO set proper values
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

  // int tmp = 20000;  // flowValue/stepSize;
  int steps = 0;
  while (!almost_equal(primalProgress, 1.0, 10) && primalProgress < 1.0)
  {
    std::cerr << "STEP: " << steps++ << std::endl;
    if (steps > 40000) break;
    gammaCouplingCheck(residualGraph, embedding, violation);

    if (isFlowValueInfeasible(residualGraph, demands, embedding, primalProgress))
    {
      std::cerr << "to much flow1111 " << flowValue << std::endl;
      return false;
    }

    if (isEarlyTerminationPossible(primalProgress, flowValue, undirectedGraph.edges.size(), etaValue))
    {
      /*
      That is, once we know that we are within an additive factor of m 2 −η of the target ﬂow, we do not
      execute progress steps anymore. Observe that in that case we can just round our current ﬂow and
      1
      compute the optimal solution using at most m 2 −η augmenting paths computation (see, e.g., [43] for
      details)
      */

      // round()
      // performAugmentingPathsComputation() m^(1/2 - etaValue) razy
    }
    // augumenting
    {
      std::cerr << "-------------------------------------------------------" << std::endl;

      primalProgress = getPrimalProgress(undirectedGraph, flow, flowValue);
      // dualProgress = sigma * y;
      std::cerr << "primalProgress: " << primalProgress << std::endl;
      std::cerr << "dualProgress: " << dualProgress << std::endl;
      std::cerr << "steps: " << steps << std::endl;

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto potentials = electricalFlow.computePotentials(undirectedGraph, demands);
      printInducedFlow(undirectedGraph, potentials, electricalFlow.resistances);
      std::cerr << "stepSize: " << geAbsoluteStepSize(primalProgress, undirectedGraph.edges.size()) << std::endl;
      std::cerr << "potentials: " << std::endl;
      for (auto elem : potentials)
      {
        std::cerr << elem << " ";
      }
      std::cerr << std::endl;

      double stepSize = geAbsoluteStepSize(primalProgress, undirectedGraph.edges.size());
      flow.update(undirectedGraph, stepSize, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, stepSize, potentials);
      printFlow(undirectedGraph, flow);
      printEmbedding(undirectedGraph, embedding);

      std::cerr << "primalProgress: " << getPrimalProgress(undirectedGraph, flow, flowValue) << std::endl;
    }

    // fixing
    {
      // gammaCouplingCheck(graph, embedding, );
      CorrectionFlow correctionFlow(residualGraph, embedding);
      printCorrections(undirectedGraph, correctionFlow);

      flow.applyCorrectionFlow(undirectedGraph, correctionFlow);
      printFlow(undirectedGraph, flow);

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto fixingDemands = Demands(undirectedGraph, correctionFlow);
      std::cerr << "fixingDemands: " << std::endl;
      for (auto node : undirectedGraph.nodes)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << fixingDemands.getDemand(node)
                  << " ";
        std::cerr << std::setprecision(ss);
      }
      auto potentials = electricalFlow.computePotentials(undirectedGraph, fixingDemands);
      printInducedFlow(undirectedGraph, potentials, electricalFlow.resistances);

      flow.update(undirectedGraph, 1.0, potentials, electricalFlow.resistances);
      embedding.update(undirectedGraph, 1.0, potentials);

      printFlow(undirectedGraph, flow);
    }
    feasibilityCheck(undirectedGraph, flow, demands);

    getchar();
  }
  return true;
}

bool MaxFlowSolver::computeMaxFlow(const Graph& directedGraph)
{
  auto undirectedGraph = UndirectedGraph::fromDirected(directedGraph);
  assert(undirectedGraph.edges.size() == 6);
  undirectedGraph.addPreconditioningEdges();
  assert(undirectedGraph.edges.size() == 12);
  // TODO
}

bool MaxFlowSolver::computeMaxFlow(const Graph& directedGraph, double flowValue)
{
  auto undirectedGraph = UndirectedGraph::fromDirected(directedGraph);
  assert(undirectedGraph.edges.size() == 6);
  undirectedGraph.addPreconditioningEdges();
  assert(undirectedGraph.edges.size() == 12);
  return computeMaxFlow(undirectedGraph, flowValue);
}