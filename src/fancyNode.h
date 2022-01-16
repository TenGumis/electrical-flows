#ifndef _FANCY_NODE_H_
#define _FANCY_NODE_H_

#include "node.h"
#include "edge.h"
#include "flowNode.h"
#include <vector>

class Edge;
class Node;
class FlowNode;

class FancyNode
{
public:

    std::vector<const Node*> fancyId; //consider set ordered by ID
    int demand = 0;
    FlowNode* correspondingFlowNode = nullptr;

    FancyNode(Edge* edge);
    FancyNode(Node* node);
};

#endif // _FANCY_NODE_H_