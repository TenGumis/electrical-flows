#ifndef _DYNAMIC_TREES_H_
#define _DYNAMIC_TREES_H_

#include "undirectedNode.h"

class DynamicTreeNode;
class UndirectedNode;

class DynamicTrees
{
 public:
  std::vector<std::shared_ptr<DynamicTreeNode>> dynamicTreesNodes;

  explicit DynamicTrees(const std::vector<std::shared_ptr<UndirectedNode>>& undirectedGraphNodes);

  void link(const DynamicTreeNode* root, const DynamicTreeNode* node, double value);
  void cut(const DynamicTreeNode* node);
  void updatePath(const DynamicTreeNode* node, double value);
  double getCost(const DynamicTreeNode* node);
  DynamicTreeNode* getMinCostNode(const DynamicTreeNode* node);
  DynamicTreeNode* getRoot(const DynamicTreeNode* node);
  DynamicTreeNode* getParent(const DynamicTreeNode* node);
};

#endif  // _DYNAMIC_TREES_H_