#ifndef _SPLAY_NODE_H_
#define _SPLAY_NODE_H_

#include "dynamicTreeNode.h"

#include <utility>

class DynamicTreeNode;

class SplayNode
{
 public:
  SplayNode* parent;
  std::pair<SplayNode*, SplayNode*> childrens;

  DynamicTreeNode* dynamicTreeEquivalent;
};

#endif  // _SPLAY_NODE_H_