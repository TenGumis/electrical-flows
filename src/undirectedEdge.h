#ifndef _UNDIRECTED_EDGE_H_
#define _UNDIRECTED_EDGE_H_

#include "edge.h"
#include "undirectedNode.h"

#include <utility>

class Edge;
class UndirectedNode;

class UndirectedEdge
{
 public:
  const std::pair<UndirectedNode*, UndirectedNode*> endpoints;
  int id;
  int capacity;
  bool isPreconditioned;
  Edge* directedEquivalent;

  UndirectedEdge(std::pair<UndirectedNode*, UndirectedNode*> endpoints,
                 int capacity,
                 int id,
                 bool isPreconditioned = 0);
};

#endif  // _UNDIRECTED_EDGE_H_