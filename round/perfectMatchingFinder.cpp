#include "perfectMatchingFinder.h"

#include <unordered_map>
#include <random>
#include <algorithm>
#include <cassert>

void PerfectMatchingFinder::find(MatchingGraph* matchingGraph)
{
    int numberNodesP = matchingGraph->nodesP.size();
    int numberOfSteps = 0;
    MatchingPairs matches;
    SuperNodePairs superNodes; // mean that key,value pair is a supernode in matching graph.
    auto randomNodesP = getRandomNodesQueue(matchingGraph->nodesP);
    while(matches.size() < numberNodesP)
    {
        int randomWalkSteps = 2*(2+(numberNodesP/(numberNodesP - numberOfSteps)));
        
        auto randomStartNode = randomNodesP.front();
        randomNodesP.pop();

        std::list<MatchingNode*> path;
        while(!truncatedWalk(randomStartNode, randomWalkSteps - 1, superNodes, path))
        {
            assert(false);
        }
        updateMatches(path, matches, superNodes);
        numberOfSteps++;
    }
}



std::queue<MatchingNode*> PerfectMatchingFinder::getRandomNodesQueue(const std::vector<std::shared_ptr<MatchingNode>>& nodes)
{
    std::vector<int> indexes(nodes.size());
    for(int i = 0; i < indexes.size(); i++)
    {
        indexes[i]=i;
    }

    std::random_device rd;
    std::mt19937 randomNumberGenerator(rd());
 
    std::shuffle(indexes.begin(), indexes.end(), randomNumberGenerator);

    std::queue<MatchingNode*> result;
    for(auto idx : indexes)
    {
        result.push(nodes[idx].get());
    }

    return result;
}

MatchingNode* PerfectMatchingFinder::sampleOutEdge(MatchingNode* startNode)
{
    //TODO
    return nullptr;
}

bool PerfectMatchingFinder::truncatedWalk(MatchingNode* startNode, int stepsLimit, SuperNodePairs& superNodes, std::list<MatchingNode*>& path)
{
    auto currentNodeQ = sampleOutEdge(startNode);
    stepsLimit--;

    while(stepsLimit > 0)
    {
        if(superNodes.find(currentNodeQ) == superNodes.end())
        {
            return true;
        }
        auto superNodeP = superNodes[currentNodeQ];
        path.push_back(superNodeP);

        auto nextNodeQ = sampleOutEdge(superNodeP);
        path.push_back(nextNodeQ);

        currentNodeQ = nextNodeQ;
        stepsLimit--;
    }
    return false;
}

void PerfectMatchingFinder::updateMatches(const std::list<MatchingNode*>& path, MatchingPairs& matches, SuperNodePairs& superNodes)
{
    auto currentNodeP = path.begin();
    bool isNodeP = true;
    while(true)
    {
        auto currentNodeQ = std::next(currentNodeP);
        matches[*currentNodeP] = *currentNodeQ;
        superNodes[*currentNodeQ] = *currentNodeP;
        
        auto nextNodeP = std::next(currentNodeQ);
        if(nextNodeP == path.end())
        {
            break;
        }
        matches.erase(*nextNodeP);
        currentNodeP = nextNodeP;
    }
}