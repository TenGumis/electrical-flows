#include "residualGraph.h"

ResidualGraph::ResidualGraph(const Graph& graph, const Flow& flow)
        : graph(graph),
          flow(flow)
{
}

double ResidualGraph::getForwardCapacity(int edgeId) const
{
  return graph.edges[edgeId]->capacity - flow.v[edgeId];
}

double ResidualGraph::getBackwardCapacity(int edgeId) const
{
  return graph.edges[edgeId]->capacity + flow.v[edgeId];
}

double ResidualGraph::getSymmetricalResidualCapacity(Edge* edge) const
{
  return std::min(getForwardCapacity(edge->id), getBackwardCapacity(edge->id));
}

int ResidualGraph::getNumberOfEdges() const
{
  return graph.edges.size();
}

Edge* ResidualGraph::getEdge(int edgeId) const
{
  return graph.edges[edgeId].get();
}
