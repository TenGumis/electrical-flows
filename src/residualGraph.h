#ifndef _RESIDUAL_GRAPH_H_
#define _RESIDUAL_GRAPH_H_

#include "flow.h"
#include "undirectedGraph.h"

#include <memory>
#include <vector>

class ResidualGraph
{
 public:
  const UndirectedGraph& graph;
  const Flow& flow;

  ResidualGraph(const UndirectedGraph& graph, const Flow& flow);

  double getForwardCapacity(const std::shared_ptr<UndirectedEdge>& edge, const UndirectedNode* const node) const;
  double getBackwardCapacity(const std::shared_ptr<UndirectedEdge>& edge, const UndirectedNode* const node) const;
  double getSymmetricalResidualCapacity(const std::shared_ptr<UndirectedEdge>& edge) const;
  int getNumberOfEdges() const;
  const std::shared_ptr<UndirectedEdge>& getEdge(int edgeId) const;
};

#endif  // _RESIDUAL_GRAPH_H_