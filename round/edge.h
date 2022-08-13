#ifndef _EDGE_H_
#define _EDGE_H_

#include "node.h"
#include "matchingEdge.h"

class Node;
class FancyNode;
class MatchingNode;

class Edge
{
public:
    Node* from;
    Node* to;
    double capacity;
    double flow;

    MatchingNode* matchingNodeP = nullptr;
    MatchingNode* matchingNodeQ = nullptr;

    Edge(Node* from, 
         Node* to,
         double capacity,
         double flow);
};

#endif // _EDGE_H_