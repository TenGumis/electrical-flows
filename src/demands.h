#ifndef _DEMANDS_H_
#define _DEMANDS_H_

#include "correctionFlow.h"
#include "undirectedGraph.h"

#include <vector>

class Demands
{
 private:
  std::vector<double> demands;

 public:
  Demands(const UndirectedGraph& graph, double flowValue);
  Demands(const UndirectedGraph& graph, const CorrectionFlow& correctionFlow);
  [[nodiscard]] double getDemand(const std::shared_ptr<UndirectedNode>& node) const;
};

#endif  // _DEMANDS_H_