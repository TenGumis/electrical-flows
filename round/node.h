#ifndef _NODE_H_
#define _NODE_H_

#include "edge.h"
#include <list>
#include <vector>

class Edge;

class Node
{
public:
    int id;
    std::vector<Edge*> outgoingEdges;
    std::vector<Edge*> incomingEdges;
    int part;
};

#endif // _NODE_H_