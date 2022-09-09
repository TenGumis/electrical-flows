#ifndef _DYNAMIC_TREE_NODE_H_
#define _DYNAMIC_TREE_NODE_H_

class UndirectedNode;
class UndirectedEdge;

class DynamicTreeNode
{
 public:
  UndirectedNode* undirectedNode;
  UndirectedEdge* undirectedEdge;

  explicit DynamicTreeNode(UndirectedNode* undirectedNode);
};

#endif  // _DYNAMIC_TREE_NODE_H_