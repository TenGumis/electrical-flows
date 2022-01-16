#include "flowNode.h"

FlowNode::FlowNode(FancyNode* fancyNode, bool isSource)
{
    
    demand = isSource ? -fancyNode->demand : fancyNode->demand;
}