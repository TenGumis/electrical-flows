#ifndef _UNDIRECTED_GRAPH_H_
#define _UNDIRECTED_GRAPH_H_

#include "graph.h"
#include "undirectedEdge.h"
#include "undirectedNode.h"

#include <memory>
#include <vector>

class UndirectedGraph
{
 public:
  std::vector<std::shared_ptr<UndirectedNode>> nodes;
  std::vector<std::shared_ptr<UndirectedEdge>> edges;
  UndirectedNode* s = nullptr;
  UndirectedNode* t = nullptr;

  static UndirectedGraph fromDirected(const Graph& graph);

  void addNode(const std::shared_ptr<UndirectedNode>& newNode);
  void addEdge(const std::shared_ptr<UndirectedEdge>& newEdge);
  void addPreconditioningEdges();

 private:
  static void updateContractedEdge(UndirectedGraph& undirectedGraph,
                                   std::vector<UndirectedEdge*>& contractedEdges,
                                   int label,
                                   std::pair<UndirectedNode*, UndirectedNode*> newEndpoints,
                                   int capacity,
                                   int& id);

  int getMaxCapacity() const;
};

#endif  // _UNDIRECTED_GRAPH_H_