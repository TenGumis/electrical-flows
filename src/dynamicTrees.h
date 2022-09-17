#ifndef _DYNAMIC_TREES_H_
#define _DYNAMIC_TREES_H_

#include "community-code/linkcuttree.h"
#include "undirectedNode.h"

#include <limits>

class DynamicTreeNode;
class UndirectedNode;
class LinkCutTree;

class DynamicTrees
{
 private:
  constexpr static const double INF = std::numeric_limits<double>::infinity();

 public:
  std::vector<std::shared_ptr<DynamicTreeNode>> dynamicTreesNodes;
  LinkCutTree linkCutTree;

  explicit DynamicTrees(const std::vector<std::shared_ptr<UndirectedNode>>& undirectedGraphNodes);

  void link(DynamicTreeNode* child, DynamicTreeNode* parent, double edgeValue);
  void cut(DynamicTreeNode* node);
  void updatePath(DynamicTreeNode* node, double value);
  double getCost(DynamicTreeNode* node);
  DynamicTreeNode* getMinCostNode(DynamicTreeNode* node);
  DynamicTreeNode* getRoot(DynamicTreeNode* node);
  DynamicTreeNode* getParent(DynamicTreeNode* node);
};

#endif  // _DYNAMIC_TREES_H_