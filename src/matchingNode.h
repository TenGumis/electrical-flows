#ifndef _MATCHING_NODE_H_
#define _MATCHING_NODE_H_

#include "matchingEdge.h"

#include <memory>
#include <vector>

class Edge;
class Node;
class MatchingEdge;

class MatchingNode
{
 public:
  int id;
  unsigned int demand = 0;
  std::vector<MatchingEdge*> edges;
  std::vector<double> probabilityPrefixSum;

  MatchingNode(int id, int demand = 0);
  double getDeficit() const;
};

#endif  // _MATCHING_NODE_H_