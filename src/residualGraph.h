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

  double getForwardCapacity(int edgeId) const;
  double getBackwardCapacity(int edgeId) const;
  double getSymmetricalResidualCapacity(Edge* edge) const;
  int getNumberOfEdges() const;
  Edge* getEdge(int edgeId) const;
};

#endif  // _RESIDUAL_GRAPH_H_