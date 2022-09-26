#ifndef _AUGMENTING_PATH_FINDER_H_
#define _AUGMENTING_PATH_FINDER_H_

#include "graph.h"
#include "integralFlow.h"

class AugmentingPathFinder
{
 public:
  static void applyAugmentingPaths(const Graph& directedGraph, IntegralFlow& flow);
};

#endif  // _AUGMENTING_PATH_FINDER_H_