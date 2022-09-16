#include "perfectMatchingFinder.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <random>
#include <unordered_map>

PerfectMatchingFinder::PerfectMatchingFinder(RandomnessProvider& randomnessProvider)
        : randomnessProvider(randomnessProvider)
{
}

void PerfectMatchingFinder::find(MatchingGraph& matchingGraph)
{
  int numberNodesP = matchingGraph.nodesP.size();
  int numberOfSteps = 0;
  MatchingPairs matches;
  MatchingPairsEdges matchedEdges;
  SuperNodePairs superNodes;  // mean that key,value pair is a supernode in
                              // matching graph.
  auto randomNodesP = getRandomNodesQueue(matchingGraph.nodesP);
  while (matches.size() < numberNodesP)
  {
    int randomWalkSteps = 2 * (2 + (numberNodesP / (numberNodesP - numberOfSteps)));

    auto randomStartNode = randomNodesP.front();
    randomNodesP.pop();

    std::list<MatchingEdge*> path;
    while (!truncatedWalk(randomStartNode, randomWalkSteps - 1, superNodes, path))
    {
      path.clear();
    }
    updateMatches(path, matches, matchedEdges, superNodes);
    numberOfSteps++;
  }
  for (const auto& match : matches)
  {
    std::cerr << match.first->id << " " << match.second->id << std::endl;
  }
  for (const auto& matchedEdges : matchedEdges)
  {
    matchedEdges.second->matched = true;
  }
}

std::queue<MatchingNode*> PerfectMatchingFinder::getRandomNodesQueue(
        const std::vector<std::shared_ptr<MatchingNode>>& nodes)
{
  std::queue<MatchingNode*> result;
  for (auto idx : randomnessProvider.getRandomPermutation(nodes.size()))
  {
    result.push(nodes[idx].get());
  }

  return result;
}

MatchingEdge* PerfectMatchingFinder::sampleOutEdge(MatchingNode* currentNode)
{
  auto edge = currentNode->edges[randomnessProvider.getRandomNumber(currentNode->edges.size())];
  return edge;  //(currentNode != edge->pNode) ? edge->pNode : edge->qNode;
}

bool PerfectMatchingFinder::truncatedWalk(MatchingNode* startNode,
                                          int stepsLimit,
                                          SuperNodePairs& superNodes,
                                          std::list<MatchingEdge*>& path)
{
  auto currentEdge = sampleOutEdge(startNode);
  auto currentNodeQ = currentEdge->qNode;
  path.push_back(currentEdge);
  stepsLimit--;

  while (stepsLimit > 0)
  {
    if (superNodes.find(currentNodeQ) == superNodes.end())
    {
      return true;
    }
    auto superNodeP = superNodes[currentNodeQ];
    auto nextEdge = sampleOutEdge(superNodeP);
    auto nextNodeQ = nextEdge->qNode;
    path.push_back(nextEdge);

    currentNodeQ = nextNodeQ;
    stepsLimit--;
  }
  return false;
}

void PerfectMatchingFinder::updateMatches(const std::list<MatchingEdge*>& path,
                                          MatchingPairs& matches,
                                          MatchingPairsEdges& matchedEdges,
                                          SuperNodePairs& superNodes)
{
  auto currentEdge = path.begin();
  while (true)
  {
    auto currentNodeQ = (*currentEdge)->qNode;
    auto currentNodeP = (*currentEdge)->pNode;
    matches[currentNodeP] = currentNodeQ;
    superNodes[currentNodeQ] = currentNodeP;
    matchedEdges[currentNodeP] = *currentEdge;

    auto nextEdge = std::next(currentEdge);
    if (nextEdge == path.end())
    {
      break;
    }
    matches.erase((*nextEdge)->pNode);
    matchedEdges.erase((*nextEdge)->pNode);
    currentEdge = nextEdge;
  }
}