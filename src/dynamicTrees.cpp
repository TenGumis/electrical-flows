#include "dynamicTrees.h"

#include "dynamicTreeNode.h"
#include "undirectedNode.h"

DynamicTrees::DynamicTrees(const std::vector<std::shared_ptr<UndirectedNode>>& undirectedGraphNodes)
        : dynamicTreesNodes(undirectedGraphNodes.size()),
          linkCutTree(undirectedGraphNodes.size())
{
  for (auto i = 0u; i < undirectedGraphNodes.size(); i++)
  {
    dynamicTreesNodes[i] = std::make_shared<DynamicTreeNode>(undirectedGraphNodes[i].get());
  }
}

void DynamicTrees::link(DynamicTreeNode* child, DynamicTreeNode* parent, double edgeValue)
{
  linkCutTree.link(child->undirectedNode->label, parent->undirectedNode->label);
  linkCutTree.setWeight(child->undirectedNode->label, edgeValue);
  linkCutTree.setWeight(getRoot(child)->undirectedNode->label, DynamicTrees::INF);
}

void DynamicTrees::cut(DynamicTreeNode* node)
{
  linkCutTree.cut(node->undirectedNode->label);
  linkCutTree.setWeight(node->undirectedNode->label, DynamicTrees::INF);
}

void DynamicTrees::updatePath(DynamicTreeNode* node, double value)
{
  linkCutTree.removeWeightInPath(value, node->undirectedNode->label);
}

double DynamicTrees::getCost(DynamicTreeNode* node)
{
  return linkCutTree.getEdgeWeight(node->undirectedNode->label);
}

DynamicTreeNode* DynamicTrees::getMinCostNode(DynamicTreeNode* node)
{
  return dynamicTreesNodes[NNode::getKey(linkCutTree.getMinEdge(node->undirectedNode->label))].get();
}

DynamicTreeNode* DynamicTrees::getRoot(DynamicTreeNode* node)
{
  return dynamicTreesNodes[NNode::getKey(linkCutTree.findRoot(node->undirectedNode->label))].get();
}

DynamicTreeNode* DynamicTrees::getParent(DynamicTreeNode* node)
{
  return dynamicTreesNodes[NNode::getKey(linkCutTree.getParent(node->undirectedNode->label))].get();
}