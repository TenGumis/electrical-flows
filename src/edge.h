#ifndef _EDGE_H_
#define _EDGE_H_

#include "node.h"

class Node;
class FancyNode;

class Edge
{
 public:
  int id;
  const Node* from;
  const Node* to;
  const int capacity;

  Edge(Node* from, Node* to, int capacity, int id);
};

#endif  // _EDGE_H_