#ifndef _FLOW_EDGE_H_
#define _FLOW_EDGE_H_

#include "flowNode.h"

class FlowEdge
{   
public:
    const FlowNode* const from;
    const FlowNode* const to;

    FlowEdge(FlowNode* from, FlowNode* to);
};

#endif // _FLOW_EDGE_H_