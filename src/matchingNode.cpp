#include "matchingNode.h"

MatchingNode::MatchingNode(int id, int demand)
        : id(id),
          demand(demand)
{
}

double MatchingNode::getDeficit() const
{
  double flow = 0;
  for (auto edge : edges)
  {
    flow += edge->flow;
  }

  return 1 - flow;
}