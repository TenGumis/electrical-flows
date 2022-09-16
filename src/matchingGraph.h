#ifndef _MATCHING_GRAPH_H_
#define _MATCHING_GRAPH_H_

#include "matchingEdge.h"
#include "matchingNode.h"

#include <memory>
#include <vector>

class Graph;
class Flow;

class MatchingGraph
{
 public:
  std::vector<std::shared_ptr<MatchingNode>> nodesP;
  std::vector<std::shared_ptr<MatchingNode>> nodesQ;

  std::vector<std::shared_ptr<MatchingEdge>> edges;

  static MatchingGraph toMatchingGraph(const Graph& directedGraph, Flow& flow, unsigned long flowValue);

  void toNonPerfectMatching();
  void toPerfectMatching();
};

#endif  // _MATCHING_GRAPH_H_