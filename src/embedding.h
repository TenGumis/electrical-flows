#ifndef _EMBEDDING_H_
#define _EMBEDDING_H_

#include "undirectedGraph.h"

#include <vector>

class Embedding
{
 private:
  std::vector<double> embedding;

 public:
  explicit Embedding(unsigned int size);

  [[nodiscard]] double getEmbedding(const std::shared_ptr<UndirectedNode>& node) const;
  [[nodiscard]] double getStretch(const std::shared_ptr<UndirectedEdge>& edge) const;
  void update(const UndirectedGraph& undirectedGraph, double stepSize, const std::vector<double>& potentials);
};

#endif  // _EMBEDDING_H_