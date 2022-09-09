#include "dynamicTrees.h"

#include "dynamicTreeNode.h"
#include "undirectedNode.h"

DynamicTrees::DynamicTrees(const std::vector<std::shared_ptr<UndirectedNode>> &undirectedGraphNodes)
        : dynamicTreesNodes(undirectedGraphNodes.size())
{
  for (auto i = 0u; i < undirectedGraphNodes.size(); i++)
  {
    dynamicTreesNodes[i] = std::make_shared<DynamicTreeNode>(undirectedGraphNodes[i].get());
  }
}

void DynamicTrees::link(const DynamicTreeNode *root, const DynamicTreeNode *node, double value)
{
  throw std::runtime_error("Not implemented");
}

void DynamicTrees::cut(const DynamicTreeNode *node)
{
  throw std::runtime_error("Not implemented");
}

void DynamicTrees::updatePath(const DynamicTreeNode *node, double value)
{
  throw std::runtime_error("Not implemented");
}

double DynamicTrees::getCost(const DynamicTreeNode *node)
{
  throw std::runtime_error("Not implemented");
}

DynamicTreeNode *DynamicTrees::getMinCostNode(const DynamicTreeNode *node)
{
  throw std::runtime_error("Not implemented");
}

DynamicTreeNode *DynamicTrees::getRoot(const DynamicTreeNode *node)
{
  throw std::runtime_error("Not implemented");
}

DynamicTreeNode *DynamicTrees::getParent(const DynamicTreeNode *node)
{
  throw std::runtime_error("Not implemented");
}
