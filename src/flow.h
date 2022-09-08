#ifndef _FLOW_H_
#define _FLOW_H_

#include "correctionFlow.h"
#include "undirectedGraph.h"

#include <vector>

class CorrectionFlow;

class Flow
{
 private:
  std::vector<double> flow;

 public:
  explicit Flow(int size);

  double getFlow(const UndirectedEdge* edge, const UndirectedNode* endpoint) const;
  double updateFlow(const UndirectedEdge* edge, const UndirectedNode* endpoint, double value);
  void update(const UndirectedGraph& undirectedGraph,
              double stepSize,
              const std::vector<double>& potentials,
              const std::vector<double>& resistances);
  void applyCorrectionFlow(const UndirectedGraph& undirectedGraph, const CorrectionFlow& correctionFlow);
  double getEnergy(const std::vector<double>& resistances);
};

#endif  // _FLOW_H_