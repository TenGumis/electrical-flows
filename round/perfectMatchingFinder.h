#ifndef _PERFECT_MATCHING_FINDER_H_
#define _PERFECT_MATCHING_FINDER_H_

#include "matchingGraph.h"
#include "randomnessProvider.h"

#include <unordered_map>
#include <vector>
#include <queue>
#include <list>

class PerfectMatchingFinder
{
public:
    PerfectMatchingFinder(RandomnessProvider& randomnessProvider);

    void find(MatchingGraph* matchingGraph, RandomnessProvider& randomnessProvider);

private:
    using MatchingPairs = std::unordered_map<MatchingNode*, MatchingNode*>;
    using SuperNodePairs = std::unordered_map<MatchingNode*, MatchingNode*>;
    RandomnessProvider& randomnessProvider;

    std::queue<MatchingNode*> getRandomNodesQueue(const std::vector<std::shared_ptr<MatchingNode>>& nodes);

    MatchingNode* sampleOutEdge(MatchingNode* currentNode);

    bool truncatedWalk(MatchingNode* matchingNode,
                              int stepsLimit,
                              SuperNodePairs& superNodes,
                              std::list<MatchingNode*>& path);

    void updateMatches(const std::list<MatchingNode*>& path,
                       MatchingPairs& matches,
                       SuperNodePairs& superNodes);
};

#endif // _PERFECT_MATCHING_FINDER_H_