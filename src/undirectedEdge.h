#ifndef _UNDIRECTED_EDGE_H_
#define _UNDIRECTED_EDGE_H_

#include "undirectedNode.h"

#include <utility>

class UndirectedNode;

class UndirectedEdge
{
 public:
  int id;
  const std::pair<UndirectedNode*, UndirectedNode*> endpoints;
  int capacity;

  UndirectedEdge(std::pair<UndirectedNode*, UndirectedNode*> endpoints, int capacity, int id);
};

#endif  // _UNDIRECTED_EDGE_H_