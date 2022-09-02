#ifndef _RESIDUAL_GRAPH_H_
#define _RESIDUAL_GRAPH_H_

#include "flow.h"
#include "graph.h"

#include <memory>
#include <vector>

class ResidualGraph
{
 public:
  const Graph& graph;
  const Flow& flow;

  ResidualGraph(const Graph& graph, const Flow& flow);

  double getForwardCapcity(int edgeId) const;
  double getBackwardCapcity(int edgeId) const;
  double getSymmetrizedResidualCapacity(Edge* edge) const;
  int getNumberOfEdges() const;
  Edge* getEdge(int edgeId) const;
};

#endif  // _RESIDUAL_GRAPH_H_