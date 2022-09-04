#ifndef _FLOW_H_
#define _FLOW_H_

#include "undirectedGraph.h"

#include <vector>

class Flow
{
 private:
  std::vector<double> flow;

 public:
  explicit Flow(int size);

  double getFlow(UndirectedEdge* edge, const UndirectedNode* const endpoint) const;
  void update(const UndirectedGraph& undirectedGraph,
              double stepSize,
              const std::vector<double>& potentials,
              const std::vector<double>& resistances);
  void correction(const UndirectedGraph& undirectedGraph, const std::vector<double>& corrections);
  double getEnergy(const std::vector<double>& resistances);
};

#endif  // _FLOW_H_