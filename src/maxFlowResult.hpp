#ifndef _MAX_FLOW_RESULT_H_
#define _MAX_FLOW_RESULT_H_

#include <vector>

struct MaxFlowResult
{
 public:
  bool isFeasible;
  unsigned long flowValue;
  std::vector<double> flow;
};

#endif  // _MAX_FLOW_RESULT_H_