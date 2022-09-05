#ifndef _MAX_FLOW_SOLVER_H_
#define _MAX_FLOW_SOLVER_H_

#include "graph.h"
#include "undirectedGraph.h"

class MaxFlowSolver
{
 public:
  static bool computeMaxFlow(const UndirectedGraph& undirectedGraph);
  static bool computeMaxFlow(const UndirectedGraph& undirectedGraph, double flowValue);
  static bool computeMaxFlow(const Graph& directedGraph);
  static bool computeMaxFlow(const Graph& directedGraph, double flowValue);
};

#endif  // _MAX_FLOW_SOLVER_H_