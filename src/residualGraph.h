#ifndef _RESIDUAL_GRAPH_H_
#define _RESIDUAL_GRAPH_H_

#include "flow.h"
#include "undirectedGraph.h"

#include <memory>
#include <vector>

class Flow;

class ResidualGraph
{
 public:
  const UndirectedGraph& graph;
  const Flow& flow;

  ResidualGraph(const UndirectedGraph& graph, const Flow& flow);

  [[nodiscard]] double getForwardCapacity(const std::shared_ptr<UndirectedEdge>& edge,
                                          const UndirectedNode* node) const;
  [[nodiscard]] double getBackwardCapacity(const std::shared_ptr<UndirectedEdge>& edge,
                                           const UndirectedNode* node) const;
  [[nodiscard]] double getSymmetricalResidualCapacity(const std::shared_ptr<UndirectedEdge>& edge) const;
  [[nodiscard]] unsigned int getNumberOfEdges() const;
};

#endif  // _RESIDUAL_GRAPH_H_