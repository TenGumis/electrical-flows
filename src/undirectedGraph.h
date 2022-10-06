#ifndef _UNDIRECTED_GRAPH_H_
#define _UNDIRECTED_GRAPH_H_

#include "graph.h"
#include "undirectedEdge.h"
#include "undirectedNode.h"

#include <memory>
#include <unordered_map>
#include <vector>

class UndirectedGraph
{
 public:
  std::vector<std::shared_ptr<UndirectedNode>> nodes;
  std::vector<std::shared_ptr<UndirectedEdge>> edges;
  UndirectedNode* source = nullptr;
  UndirectedNode* target = nullptr;
  std::vector<UndirectedEdge*> sourceEdges;
  std::vector<UndirectedEdge*> targetEdges;

  static UndirectedGraph fromDirected(const Graph& graph);

  void addNode(const std::shared_ptr<UndirectedNode>& newNode);
  void addEdge(const std::shared_ptr<UndirectedEdge>& newEdge);
  void addPreconditioningEdges();
  void removePreconditioningEdges();
  [[nodiscard]] unsigned long getMaxCapacity() const;
  bool containsOppositeEquivalent(const Edge* edge) const;
  UndirectedEdge* getMappedEquivalent(const Edge* edge) const;

 private:
  std::vector<std::unordered_map<unsigned int, UndirectedEdge*>> edgeMapping;

  static void updateContractedEdge(UndirectedGraph& undirectedGraph,
                                   std::vector<UndirectedEdge*>& contractedEdges,
                                   int label,
                                   std::pair<UndirectedNode*, UndirectedNode*> newEndpoints,
                                   int capacity,
                                   unsigned int& edgeIdCounter);
  static void addSourceTargetEdges(UndirectedGraph& undirectedGraph,
                                   std::pair<UndirectedNode*, UndirectedNode*>& endpoints,
                                   unsigned int capacity,
                                   unsigned int& edgeIdCounter);
  void setSourceAndTarget(UndirectedNode* source, UndirectedNode* newTarget);
};

#endif  // _UNDIRECTED_GRAPH_H_