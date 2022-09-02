#ifndef _NODE_H_
#define _NODE_H_

#include "edge.h"
#include "undirectedNode.h"

#include <list>
#include <vector>

class Edge;

class Node
{
 public:
  const int label;

  std::vector<Edge*> outgoing;
  std::vector<Edge*> incoming;
  UndirectedNode* undirectedEquivalent;

  Node(int label);
};

#endif  // _NODE_H_