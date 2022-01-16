#ifndef _EDGE_H_
#define _EDGE_H_

#include "node.h"
#include "matchingEdge.h"

class Node;
class FancyNode;

class Edge
{
public:
    int id;
    const Node* from;
    const Node* to;
    const int capacity;
    double forwardCapacity;
    double backwardCapacity;

    FancyNode* matchingNodeP;
    FancyNode* matchingNodeQ;  

    Edge(Node* from, Node* to, int capacity, int id);
};

#endif // _EDGE_H_