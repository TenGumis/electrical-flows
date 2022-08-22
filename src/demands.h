#ifndef _DEMANDS_H_
#define _DEMANDS_H_

#include "graph.h"

#include <vector>

class Demands
{
 private:
  std::vector<double> demands;

 public:
  Demands(const Graph& graph, double flowValue);
  Demands(const Graph& graph, std::vector<double>& corrections);
  double getDemand(Node* node) const;
};

#endif  // _DEMANDS_H_