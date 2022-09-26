#include "embedding.h"

Embedding::Embedding(unsigned int size)
        : embedding(size)
{
}

double Embedding::getEmbedding(const std::shared_ptr<UndirectedNode>& node) const
{
  return embedding[node->label];
}

double Embedding::getStretch(const std::shared_ptr<UndirectedEdge>& edge) const
{
  return embedding[edge->endpoints.second->label] - embedding[edge->endpoints.first->label];
}

void Embedding::update(const UndirectedGraph& undirectedGraph, double stepSize, const std::vector<double>& potentials)
{
  for (const auto& node : undirectedGraph.nodes)
  {
    embedding[node->label] += stepSize * potentials[node->label];
  }
}
