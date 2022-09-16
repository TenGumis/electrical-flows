#ifndef _MAX_FLOW_SOLVER_H_
#define _MAX_FLOW_SOLVER_H_

#include "graph.h"
#include "maxFlowResult.hpp"
#include "undirectedGraph.h"

class MaxFlowSolver
{
 public:
  static MaxFlowResult computeMaxFlow(const Graph& directedGraph);
  static MaxFlowResult computeMaxFlow(const Graph& directedGraph, unsigned long flowValue);

 private:
  static MaxFlowResult computeMaxFlow(UndirectedGraph& undirectedGraph);
  static MaxFlowResult computeMaxFlow(UndirectedGraph& undirectedGraph, unsigned long flowValue);
  static MaxFlowResult computeMaxFlowWithPreconditioning(const UndirectedGraph& directedGraph, unsigned long flowValue);
  static bool containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void removeFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void getDirectedFractionalFlow(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void roundFlow(const Graph& directedGraph, Flow& flow, unsigned long flowValue);
};

#endif  // _MAX_FLOW_SOLVER_H_