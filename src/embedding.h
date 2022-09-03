#ifndef _EMBEDDING_H_
#define _EMBEDDING_H_

#include "graph.h"

#include <vector>

class Embedding
{
 public:
  std::vector<double> v;

  explicit Embedding(int size);

  void update(const Graph& graph, double stepSize, const std::vector<double>& potentials);
};

#endif  // _EMBEDDING_H_