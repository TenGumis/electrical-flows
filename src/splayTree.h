#ifndef _SPLAY_TREE_H_
#define _SPLAY_TREE_H_

#include "splayNode.h"

#include <utility>

class DynamicTreeNode;
class SplayNode;

class SplayTree
{
 public:
  static void splay(SplayNode* splayNode);
  static void rotate(SplayNode* splayNode);
  static std::pair<SplayNode*, SplayNode*> split(SplayNode* splayNode);
  static void attach(SplayNode* parent, SplayNode* child, bool side);
};

#endif  // _SPLAY_TREE_H_