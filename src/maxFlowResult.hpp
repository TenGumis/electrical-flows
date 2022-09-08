#ifndef _MAX_FLOW_RESULT_H_
#define _MAX_FLOW_RESULT_H_

#include "flow.h"

struct MaxFlowResult
{
 public:
  bool isFeasible;
  unsigned long flowValue;
  Flow flow;
};

#endif  // _MAX_FLOW_RESULT_H_