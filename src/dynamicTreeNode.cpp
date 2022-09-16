#include "dynamicTreeNode.h"

#include "undirectedNode.h"

DynamicTreeNode::DynamicTreeNode(UndirectedNode* undirectedNode)
        : undirectedNode(undirectedNode),
          splayParent(nullptr),
          splayChildrens(std::make_pair(nullptr, nullptr)),
          cost(0.0),
          deltaCost(0.0),
          minCostNode(nullptr),
          pathParent(nullptr),
          undirectedEdge(nullptr)
{
}
