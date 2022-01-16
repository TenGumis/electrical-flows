#include "fancyNode.h"    
    
FancyNode::FancyNode(Edge* edge) 
{
    fancyId.push_back(edge->from);
    fancyId.push_back(edge->to);
    demand = edge->capacity; 
}

FancyNode::FancyNode(Node* node) 
{
    fancyId.push_back(node);
    demand = 0;
}