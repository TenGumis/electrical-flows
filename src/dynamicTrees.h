#ifndef _DYNAMIC_TREES_H_
#define _DYNAMIC_TREES_H_

#include "tmp/linkcuttree.h"
#include "undirectedNode.h"

class DynamicTreeNode;
class UndirectedNode;
class LinkCutTree;

class DynamicTrees
{
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

 private:
  void detachRightChildFromPath(DynamicTreeNode* node);
  void access(DynamicTreeNode* node);

  void splay(DynamicTreeNode* node);
  void rotate(DynamicTreeNode* node);
  void attach(DynamicTreeNode* parent, DynamicTreeNode* child, bool side);
  bool isRightChild(DynamicTreeNode* node) const;
};

#endif  // _DYNAMIC_TREES_H_