#ifndef _DYNAMIC_TREE_NODE_H_
#define _DYNAMIC_TREE_NODE_H_

#include <utility>

class UndirectedNode;
class UndirectedEdge;

class DynamicTreeNode
{
  friend class DynamicTrees;

 private:
  DynamicTreeNode* splayParent;
  std::pair<DynamicTreeNode*, DynamicTreeNode*> splayChildrens;

  double cost;
  double deltaCost;
  DynamicTreeNode* minCostNode;

  DynamicTreeNode* pathParent;

 public:
  UndirectedNode* undirectedNode;
  UndirectedEdge* undirectedEdge;

  explicit DynamicTreeNode(UndirectedNode* undirectedNode);
};

#endif  // _DYNAMIC_TREE_NODE_H_