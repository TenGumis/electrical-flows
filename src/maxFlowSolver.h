#ifndef _MAX_FLOW_SOLVER_H_
#define _MAX_FLOW_SOLVER_H_

#include "graph.h"
#include "integralFlow.h"
#include "maxFlowResult.hpp"
#include "undirectedGraph.h"

class MaxFlowSolver
{
 public:
  static MaxFlowResult computeMaxFlow(const Graph& directedGraph);

 private:
  static MaxFlowResult computeMaxFlowWithPreconditioning(const UndirectedGraph& directedGraph, unsigned long flowValue);
  static bool containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void removeFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void getDirectedFractionalFlow(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static unsigned long getFlowValue(const Graph& directedGraph, const Flow& flow);
  static void applyAugmentingPaths(const Graph& directedGraph, IntegralFlow& flow);
  static bool findAugmentingPath(const Graph& directedGraph, IntegralFlow& flow);
};

#endif  // _MAX_FLOW_SOLVER_H_