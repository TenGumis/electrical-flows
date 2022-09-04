#ifndef _VIOLATION_H_
#define _VIOLATION_H_

#include "graph.h"

#include <vector>

class Violation
{
 private:
 public:
  std::vector<double> violation;

  explicit Violation(int size);

  double getViolation(const std::shared_ptr<UndirectedEdge>& edge) const;
};

#endif  // _VIOLATION_H_