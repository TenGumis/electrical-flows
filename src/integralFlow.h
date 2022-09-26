#ifndef _INTEGRAL_FLOW_H_
#define _INTEGRAL_FLOW_H_

#include "correctionFlow.h"
#include "undirectedGraph.h"

#include <vector>

class IntegralFlow
{
 private:
  std::vector<unsigned long> flow;

 public:
  explicit IntegralFlow(unsigned int size);

  unsigned long getFlow(const Edge* edge) const;
  void setFlow(const Edge* edge, unsigned long value);
  void updateFlow(const Edge* edge, unsigned long value);
};

#endif  // _INTEGRAL_FLOW_H_