#include "residualGraph.h"

ResidualGraph::ResidualGraph(const Graph& graph, const Flow& flow)
        : graph(graph),
          flow(flow)
{
}

double ResidualGraph::getForwardCapcity(int edgeId) const
{
  return graph.edges[edgeId]->capacity - flow.v[edgeId];
}

double ResidualGraph::getBackwardCapcity(int edgeId) const
{
  return graph.edges[edgeId]->capacity + flow.v[edgeId];
}

double ResidualGraph::getSymmetrizedResidualCapacity(Edge* edge) const
{
  return std::min(getForwardCapcity(edge->id), getBackwardCapcity(edge->id));
}

int ResidualGraph::getNumberOfEdges() const
{
  return graph.edges.size();
}

Edge* ResidualGraph::getEdge(int edgeId) const
{
  return graph.edges[edgeId].get();
}
