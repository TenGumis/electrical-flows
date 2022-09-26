#ifndef _MAX_FLOW_SOLVER_H_
#define _MAX_FLOW_SOLVER_H_

#include "demands.h"
#include "graph.h"
#include "integralFlow.h"
#include "maxFlowResult.hpp"
#include "undirectedGraph.h"

class MaxFlowSolver
{
 public:
  static MaxFlowResult computeMaxFlow(const Graph& directedGraph);

 private:
  [[maybe_unused]] static bool gammaCouplingCheck(const ResidualGraph& residualGraph, const Embedding& embedding);
  static double getAbsoluteStepSize(double primalProgress, unsigned int numberOfEdges);
  static double getPrimalProgress(const UndirectedGraph& undirectedGraph, const Flow& flow, double flowValue);
  static double getEtaValue(unsigned int maxCapacity, unsigned int numberOfEdges);
  static bool isEarlyTerminationPossible(double primalProgress,
                                         double flowValue,
                                         unsigned int numberOfEdges,
                                         double etaValue);
  static bool isFlowValueInfeasible(const ResidualGraph& residualGraph,
                                    const Demands& demands,
                                    const Embedding& embedding,
                                    double primalProgress);
  static MaxFlowResult computeMaxFlowWithPreconditioning(const UndirectedGraph& directedGraph, unsigned long flowValue);

  static bool containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void removeFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void getDirectedFractionalFlow(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static unsigned long getFlowValue(const Graph& directedGraph, const Flow& flow);
  static void applyAugmentingPaths(const Graph& directedGraph, IntegralFlow& flow);
  static bool findAugmentingPath(const Graph& directedGraph, IntegralFlow& flow);
};

#endif  // _MAX_FLOW_SOLVER_H_