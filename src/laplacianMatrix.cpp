#include "laplacianMatrix.h"

#include <cassert>

LaplacianMatrix::LaplacianMatrix(const Graph& graph, const std::vector<double>& resistances)
        : size(graph.nodes.size()),
          v(graph.nodes.size(), std::vector<double>(graph.nodes.size(), 0.0))
{
  assert(graph.edges.size() == resistances.size());

  for (const auto edge : graph.edges)
  {
    auto u = edge->from->label;
    auto v = edge->to->label;
    this->v[u][v] = -1 / resistances[edge->id];  // with weights given by the (inverses of) the resistances r
    this->v[v][u] = -1 / resistances[edge->id];  // maybe one of them should be positive?
  }

  for (int i = 0; i < graph.nodes.size(); i++)
  {
    for (const auto edge : graph.nodes[i]->incoming)
    {
      this->v[i][i] += 1 / resistances[edge->id];
    }
    for (const auto edge : graph.nodes[i]->outgoing)
    {
      this->v[i][i] += 1 / resistances[edge->id];
    }
  }
}
