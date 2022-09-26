#ifndef _MAX_FLOW_RESULT_H_
#define _MAX_FLOW_RESULT_H_

#include "flow.h"
#include "integralFlow.h"

struct MaxFlowResult
{
 public:
  bool isFeasible;
  Flow flow;
  IntegralFlow integralFlow = IntegralFlow(0);
};

#endif  // _MAX_FLOW_RESULT_H_