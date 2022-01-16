#ifndef _FLOW_NODE_H_
#define _FLOW_NODE_H_

#include "fancyNode.h"

class FancyNode;

class FlowNode
{
public:
    int demand = 0;

    FlowNode(FancyNode* fancyNode, bool isSource);
    FlowNode() = default;
};

#endif // _FLOW_NODE_H_