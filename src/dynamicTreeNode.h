#ifndef _DYNAMIC_TREE_NODE_H_
#define _DYNAMIC_TREE_NODE_H_

#include <utility>

class UndirectedNode;
class UndirectedEdge;

class DynamicTreeNode
{
  friend class DynamicTrees;

 public:
  UndirectedNode* undirectedNode;
  UndirectedEdge* undirectedEdge;

  explicit DynamicTreeNode(UndirectedNode* undirectedNode);
};

#endif  // _DYNAMIC_TREE_NODE_H_