#include "embedding.h"

Embedding::Embedding(int size)
        : v(size)
{
}

void Embedding::update(const Graph& graph, double stepSize, const std::vector<double>& potentials)
{
  for (const auto& node : graph.nodes)
  {
    v[node->label] += stepSize * potentials[node->label];
  }
}
