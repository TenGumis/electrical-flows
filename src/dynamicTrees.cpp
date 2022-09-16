#include "dynamicTrees.h"

#include "dynamicTreeNode.h"
#include "undirectedNode.h"

#include <cassert>
#include <limits>
/*
DynamicTrees::DynamicTrees(const std::vector<std::shared_ptr<UndirectedNode>>& undirectedGraphNodes)
        : dynamicTreesNodes(undirectedGraphNodes.size())
{
  for (auto i = 0u; i < undirectedGraphNodes.size(); i++)
  {
    dynamicTreesNodes[i] = std::make_shared<DynamicTreeNode>(undirectedGraphNodes[i].get());
  }
}

void DynamicTrees::link(DynamicTreeNode* child, DynamicTreeNode* parent, double edgeValue)
{
  auto parentMinCostNodeCost = getCost(parent->minCostNode);

  access(child);
  access(parent);
  child->splayChildrens.first = parent;
  parent->splayParent = child;

  child->cost = edgeValue;
  child->minCostNode = (edgeValue < parentMinCostNodeCost) ? child : parent->minCostNode;
}

void DynamicTrees::cut(DynamicTreeNode* node)
{
  access(node);
  node->splayChildrens.first->splayParent = nullptr;
  node->splayChildrens.first= nullptr;
}

void DynamicTrees::updatePath(DynamicTreeNode* node, double value)
{
  access(node);
  node->deltaCost += value;
}

double DynamicTrees::getCost(DynamicTreeNode* node)
{
  access(node);
  return node->cost + node->deltaCost;
}

DynamicTreeNode* DynamicTrees::getMinCostNode(DynamicTreeNode* node)
{
  access(node);
  return node->minCostNode;
}

DynamicTreeNode* DynamicTrees::getRoot(DynamicTreeNode* node)
{
  access(node);
  while(node->splayChildrens.first)
  {
    node = node->splayChildrens.first;
  }
  splay(node);

  return node;
}

DynamicTreeNode* DynamicTrees::getParent(DynamicTreeNode* node)
{
  access(node);
  auto nextNode = node->splayChildrens.first;
  while(nextNode->splayChildrens.second)
  {
    nextNode = nextNode->splayChildrens.second;
  }
  splay(nextNode);

  return nextNode;
}

*/

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
  double INF = std::numeric_limits<double>::infinity();
  linkCutTree.setWeight(getRoot(child)->undirectedNode->label, INF);
}

void DynamicTrees::cut(DynamicTreeNode* node)
{
  linkCutTree.cut(node->undirectedNode->label);
  double INF = std::numeric_limits<double>::infinity();
  linkCutTree.setWeight(node->undirectedNode->label, INF);
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

void DynamicTrees::detachRightChildFromPath(DynamicTreeNode* node)
{
  if (node->splayChildrens.second)
  {
    auto child = node->splayChildrens.second;
    child->pathParent = node;
    child->splayParent = nullptr;
    node->splayChildrens.second = nullptr;
  }
}

void DynamicTrees::access(DynamicTreeNode* node)
{
  splay(node);
  detachRightChildFromPath(node);

  while (node->pathParent != nullptr)
  {
    DynamicTreeNode* parent = node->pathParent;
    splay(parent);
    detachRightChildFromPath(parent);
    attach(parent, node, true);

    node->pathParent = nullptr;
    splay(node);
  }
}

void DynamicTrees::splay(DynamicTreeNode* node)
{
  while (node->splayParent)
  {
    if (node->splayParent->splayParent)
    {
      if (isRightChild(node) == isRightChild(node->splayParent))
      {
        rotate(node->splayParent);
      }
      else
      {
        rotate(node);
      }
    }
    rotate(node);
  }
}

void DynamicTrees::rotate(DynamicTreeNode* node)
{
  const auto oldParent = node->splayParent;
  const auto oldSide = isRightChild(node);

  if (node->splayParent->splayParent)
  {
    attach(node->splayParent->splayParent, node, isRightChild(node->splayParent));
  }
  else
  {
    node->splayParent = nullptr;
  }

  auto childToRelink = oldSide ? node->splayChildrens.first : node->splayChildrens.second;
  attach(oldParent, childToRelink, oldSide);
  attach(node, oldParent, !oldSide);
  node->pathParent = oldParent->pathParent;
}

void DynamicTrees::attach(DynamicTreeNode* parent, DynamicTreeNode* child, bool rightSide)
{
  assert(parent);

  if (rightSide)
  {
    parent->splayChildrens.second = child;
  }
  else
  {
    parent->splayChildrens.first = child;
  }
  if (child)
  {
    child->splayParent = parent;
  }
}

bool DynamicTrees::isRightChild(DynamicTreeNode* node) const
{
  return node->splayParent->splayChildrens.second == node;
}