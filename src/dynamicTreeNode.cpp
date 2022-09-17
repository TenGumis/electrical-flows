#include "dynamicTreeNode.h"

#include "undirectedNode.h"

DynamicTreeNode::DynamicTreeNode(UndirectedNode* undirectedNode)
        : undirectedNode(undirectedNode),
          undirectedEdge(nullptr)
{
}
