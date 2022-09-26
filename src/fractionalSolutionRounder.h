#ifndef _FRACTIONAL_SOLUTION_ROUNDER_H_
#define _FRACTIONAL_SOLUTION_ROUNDER_H_

#include "flow.h"
#include "graph.h"
#include "integralFlow.h"

class FractionalSolutionRounder
{
 private:
 public:
  static IntegralFlow roundFlow(const Graph& directedGraph, Flow& flow, unsigned long flowValue);
};

#endif  // _FRACTIONAL_SOLUTION_ROUNDER_H_