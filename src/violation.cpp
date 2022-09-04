#include "violation.h"

Violation::Violation(int size)
        : violation(size)
{
}

double Violation::getViolation(const std::shared_ptr<UndirectedEdge>& edge) const
{
  return violation[edge->id];
}
