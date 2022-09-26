#ifndef _PERFECT_MATCHING_FINDER_H_
#define _PERFECT_MATCHING_FINDER_H_

#include "matchingGraph.h"
#include "randomnessProvider.h"

#include <list>
#include <queue>
#include <unordered_map>
#include <vector>

class PerfectMatchingFinder
{
 public:
  explicit PerfectMatchingFinder(RandomnessProvider& randomnessProvider);

  void find(MatchingGraph& matchingGraph);

 private:
  using MatchingPairs = std::unordered_map<MatchingNode*, MatchingNode*>;
  using MatchingPairsEdges = std::unordered_map<MatchingNode*, MatchingEdge*>;
  using SuperNodePairs = std::unordered_map<MatchingNode*, MatchingNode*>;
  RandomnessProvider& randomnessProvider;

  static void initializeProbabilityPrefixSum(MatchingGraph& matchingGraph);

  std::queue<MatchingNode*> getRandomNodesQueue(const std::vector<std::shared_ptr<MatchingNode>>& nodes);
  MatchingEdge* sampleOutEdge(MatchingNode* currentNode);
  bool truncatedWalk(MatchingNode* matchingNode,
                     unsigned int stepsLimit,
                     SuperNodePairs& superNodes,
                     std::list<MatchingEdge*>& path);
  static void updateMatches(const std::list<MatchingEdge*>& path,
                            MatchingPairs& matches,
                            MatchingPairsEdges& matchedEdges,
                            SuperNodePairs& superNodes);
};

#endif  // _PERFECT_MATCHING_FINDER_H_