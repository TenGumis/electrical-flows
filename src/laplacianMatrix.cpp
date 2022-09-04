#include "laplacianMatrix.h"

#include <cassert>

LaplacianMatrix::LaplacianMatrix(const UndirectedGraph& undirectedGraph, const std::vector<double>& resistances)
        : size(undirectedGraph.nodes.size()),
          matrix(undirectedGraph.nodes.size(), std::vector<double>(undirectedGraph.nodes.size(), 0.0))
{
  assert(undirectedGraph.edges.size() == resistances.size());

  for (const auto& edge : undirectedGraph.edges)
  {
    auto u = edge->endpoints.first->label;
    auto v = edge->endpoints.second->label;
    matrix[u][v] = -1 / resistances[edge->id];  // with weights given by the (inverses of) the resistances r
    matrix[v][u] = -1 / resistances[edge->id];  // maybe one of them should be positive?
  }

  for (int i = 0; i < undirectedGraph.nodes.size(); i++)
  {
    for (const auto edge : undirectedGraph.nodes[i]->incident)
    {
      matrix[i][i] += 1 / resistances[edge->id];
    }
  }
}
