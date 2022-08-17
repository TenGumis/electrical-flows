#ifndef _PERFECT_MATCHING_FINDER_H_
#define _PERFECT_MATCHING_FINDER_H_

#include "matchingGraph.h"

#include <unordered_map>
#include <vector>
#include <queue>
#include <list>

class PerfectMatchingFinder
{
public:
    static void find(MatchingGraph* matchingGraph);

private:
    using MatchingPairs = std::unordered_map<MatchingNode*, MatchingNode*>;
    using SuperNodePairs = std::unordered_map<MatchingNode*, MatchingNode*>;

    static std::queue<MatchingNode*> getRandomNodesQueue(const std::vector<std::shared_ptr<MatchingNode>>& nodes);
    static MatchingNode* sampleOutEdge(MatchingNode* startNode);
    static bool truncatedWalk(MatchingNode* matchingNode, int stepsLimit, SuperNodePairs& superNodes, std::list<MatchingNode*>& path);
    static void updateMatches(const std::list<MatchingNode*>& path, MatchingPairs& matches, SuperNodePairs& superNodes);
};

#endif // _PERFECT_MATCHING_FINDER_H_