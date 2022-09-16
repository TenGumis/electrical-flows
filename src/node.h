#ifndef _NODE_H_
#define _NODE_H_

#include "edge.h"
#include "undirectedNode.h"

#include <list>
#include <vector>

class Edge;
class UndirectedNode;

class Node
{
 public:
  const int label;

  std::vector<Edge*> outgoingEdges;
  std::vector<Edge*> incomingEdges;
  UndirectedNode* undirectedEquivalent;

  Node(int label);
};

#endif  // _NODE_H_