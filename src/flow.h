#ifndef _FLOW_H_
#define _FLOW_H_

#include "graph.h"

#include <vector>

class Flow
{
 public:
  std::vector<double> v;

  Flow(int size);

  void update(const Graph& graph,
              double stepSize,
              const std::vector<double>& potentials,
              const std::vector<double>& resistances);
  void correction(const Graph& graph, const std::vector<double>& corrections);
  double getEnergy(const std::vector<double>& resistances);
};

#endif  // _FLOW_H_