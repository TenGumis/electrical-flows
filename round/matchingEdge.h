#ifndef _MATCHING_EDGE_H_
#define _MATCHING_EDGE_H_

#include "matchingNode.h"

class MatchingNode;

class MatchingEdge
{
 public:
  MatchingNode* pNode;
  MatchingNode* qNode;
  double flow;

  MatchingEdge(MatchingNode* pNode, MatchingNode* qNode, double flow = 0.0);
};

#endif  // _MATCHING_EDGE_H_