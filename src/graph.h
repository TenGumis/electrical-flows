#ifndef _GRAPH_H_
#define _GRAPH_H_

#include "edge.h"
#include "node.h"

#include <memory>
#include <vector>

class Graph
{
 public:
  std::vector<std::shared_ptr<Node>> nodes;
  std::vector<std::shared_ptr<Edge>> edges;
  Node* s = nullptr;
  Node* t = nullptr;

  void addNode(std::shared_ptr<Node> newNode);
  void addEdge(std::shared_ptr<Edge> newEdge);
};

#endif  // _GRAPH_H_