#include "integralFlow.h"

#include "correctionFlow.h"

IntegralFlow::IntegralFlow(int size)
        : flow(size)
{
}

unsigned long IntegralFlow::getFlow(const Edge* const edge) const
{
  return flow[edge->id];
}

void IntegralFlow::setFlow(const Edge* edge, unsigned long value)
{
  flow[edge->id] = value;
}

void IntegralFlow::updateFlow(const Edge* edge, unsigned long value)
{
  flow[edge->id] += value;
}
