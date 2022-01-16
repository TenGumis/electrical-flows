#ifndef _NODE_H_
#define _NODE_H_

#include "edge.h"
#include <list>
#include <vector>

class Edge;

class Node
{
public:
    const int label;

    std::vector<Edge*> outgoing;
    std::vector<Edge*> incoming;
    

    Node(int label);
};

#endif // _NODE_H_