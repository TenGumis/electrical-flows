#ifndef _MATCHING_NODE_H_
#define _MATCHING_NODE_H_

#include "matchingEdge.h"

#include <vector>
#include <memory>

class Edge;
class Node;
class MatchingEdge;

class MatchingNode
{
public:

    int id;
    int demand = 0;
    std::vector<MatchingEdge*> edges;

    MatchingNode(int id, int demand = 0);
};

#endif // _MATCHING_NODE_H_