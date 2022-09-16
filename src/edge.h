#ifndef _EDGE_H_
#define _EDGE_H_

#include "matchingEdge.h"
#include "matchingNode.h"
#include "node.h"
#include "undirectedEdge.h"

class Node;
class UndirectedEdge;

class Edge
{
 public:
  int id;
  const Node* from;
  const Node* to;
  const int capacity;

  UndirectedEdge* undirectedEquivalent = nullptr;
  MatchingEdge* matchingEquivalent = nullptr;

  Edge(Node* from, Node* to, int capacity, int id);
};

#endif  // _EDGE_H_