#ifndef _CORRECTION_FLOW_H_
#define _CORRECTION_FLOW_H_

#include "embedding.h"

#include <vector>

class ResidualGraph;

class CorrectionFlow
{
 private:
  std::vector<double> correction;

 public:
  explicit CorrectionFlow(const ResidualGraph& residualGraph, const Embedding& embedding);
  [[nodiscard]] double getCorrectionFlow(const std::shared_ptr<UndirectedEdge>& edge) const;
  [[nodiscard]] double getCorrectionFlow(const UndirectedEdge* edge) const;
};

#endif  // _CORRECTION_FLOW_H_