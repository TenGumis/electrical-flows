#include "residualGraph.h"

#include "flow.h"

ResidualGraph::ResidualGraph(const UndirectedGraph& graph, const Flow& flow)
        : graph(graph),
          flow(flow)
{
}

double ResidualGraph::getForwardCapacity(const std::shared_ptr<UndirectedEdge>& edge,
                                         const UndirectedNode* const node) const
{
  return edge->capacity - flow.getFlow(edge.get(), node);
}

double ResidualGraph::getBackwardCapacity(const std::shared_ptr<UndirectedEdge>& edge,
                                          const UndirectedNode* const node) const
{
  return edge->capacity + flow.getFlow(edge.get(), node);
}

double ResidualGraph::getSymmetricalResidualCapacity(const std::shared_ptr<UndirectedEdge>& edge) const
{
  return std::min(getForwardCapacity(edge, edge->endpoints.first), getBackwardCapacity(edge, edge->endpoints.first));
}

unsigned int ResidualGraph::getNumberOfEdges() const
{
  return graph.edges.size();
}
