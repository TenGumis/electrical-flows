#ifndef _UNDIRECTED_NODE_H_
#define _UNDIRECTED_NODE_H_

#include "undirectedEdge.h"

#include <list>
#include <vector>

class UndirectedEdge;

class UndirectedNode
{
 public:
  const int label;
  std::vector<UndirectedEdge*> incident;

  UndirectedNode(int label);
};

#endif  // _UNDIRECTED_NODE_H_