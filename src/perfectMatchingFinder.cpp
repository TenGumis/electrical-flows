#include "perfectMatchingFinder.h"

#include <cassert>
#include <random>
#include <unordered_set>

PerfectMatchingFinder::PerfectMatchingFinder(RandomnessProvider& randomnessProvider)
        : randomnessProvider(randomnessProvider)
{
}

void PerfectMatchingFinder::initializeProbabilityPrefixSum(MatchingGraph& matchingGraph)
{
  for (auto& node : matchingGraph.nodesP)
  {
    node->probabilityPrefixSum.resize(node->edges.size());
    double prefixSum = 0.0;
    for (unsigned int i = 0U; i < node->edges.size(); i++)
    {
      prefixSum += node->edges[i]->flow;
      node->probabilityPrefixSum[i] = prefixSum;
    }
    node->probabilityPrefixSum.back() = std::numeric_limits<double>::infinity();
  }
  for (auto& node : matchingGraph.nodesQ)
  {
    node->probabilityPrefixSum.resize(node->edges.size());
    double prefixSum = 0.0;
    for (unsigned int i = 0U; i < node->edges.size(); i++)
    {
      prefixSum += node->edges[i]->flow;
      node->probabilityPrefixSum[i] = prefixSum;
    }
    node->probabilityPrefixSum.back() = std::numeric_limits<double>::infinity();
  }
}

void PerfectMatchingFinder::find(MatchingGraph& matchingGraph)
{
  initializeProbabilityPrefixSum(matchingGraph);
  auto numberNodesP = matchingGraph.nodesP.size();
  auto numberOfSteps = 0u;
  MatchingPairs matches;
  MatchingPairsEdges matchedEdges;
  SuperNodePairs superNodes;  // mean that key,value pair is a supernode in
                              // matching graph.
  auto randomNodesP = getRandomNodesQueue(matchingGraph.nodesP);
  while (matches.size() < numberNodesP)
  {
    auto randomWalkSteps = 2 * (2 + (numberNodesP / (numberNodesP - numberOfSteps)));

    auto randomStartNode = randomNodesP.front();
    randomNodesP.pop();

    std::list<MatchingEdge*> path;
    while (!truncatedWalk(randomStartNode, randomWalkSteps - 1, superNodes, path))
    {
      path.clear();
    }
    path = removePathCycles(path);
    updateMatches(path, matches, matchedEdges, superNodes);
    numberOfSteps++;
  }
  for (const auto& matchedEdge : matchedEdges)
  {
    matchedEdge.second->matched = true;
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
  auto randomNumber = randomnessProvider.getRandomNumber();
  auto prefixSumPosition = std::lower_bound(
          currentNode->probabilityPrefixSum.begin(), currentNode->probabilityPrefixSum.end(), randomNumber);
  auto edgeIdx = std::distance(currentNode->probabilityPrefixSum.begin(), prefixSumPosition);
  auto edge = currentNode->edges[edgeIdx];
  return edge;  //(currentNode != edge->pNode) ? edge->pNode : edge->qNode;
}

bool PerfectMatchingFinder::truncatedWalk(MatchingNode* startNode,
                                          unsigned int stepsLimit,
                                          const SuperNodePairs& superNodes,
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
    auto superNodeP = superNodes.at(currentNodeQ);
    auto nextEdge = sampleOutEdge(superNodeP);
    auto nextNodeQ = nextEdge->qNode;
    path.push_back(nextEdge);

    currentNodeQ = nextNodeQ;
    stepsLimit--;
  }
  return false;
}

std::list<MatchingEdge*> PerfectMatchingFinder::removePathCycles(const std::list<MatchingEdge*>& path)
{
  std::list<MatchingEdge*> result;
  std::unordered_set<MatchingNode*> visitedNodeQ;
  for (const auto edge : path)
  {
    if (visitedNodeQ.contains(edge->qNode))
    {
      while (result.back()->qNode != edge->qNode)
      {
        visitedNodeQ.erase(result.back()->qNode);
        result.pop_back();
      }
    }
    else
    {
      visitedNodeQ.insert(edge->qNode);
      result.push_back(edge);
    }
  }

  return result;
}

void PerfectMatchingFinder::updateMatches(const std::list<MatchingEdge*>& path,
                                          MatchingPairs& matches,
                                          MatchingPairsEdges& matchedEdges,
                                          SuperNodePairs& superNodes)
{
  auto currentEdge = path.begin();
  while (currentEdge != path.end())
  {
    matches.erase((*currentEdge)->pNode);
    superNodes.erase((*currentEdge)->qNode);
    matchedEdges.erase((*currentEdge)->pNode);

    auto currentNodeQ = (*currentEdge)->qNode;
    auto currentNodeP = (*currentEdge)->pNode;

    matches[currentNodeP] = currentNodeQ;
    superNodes[currentNodeQ] = currentNodeP;
    matchedEdges[currentNodeP] = *currentEdge;

    currentEdge = std::next(currentEdge);
  }
}
