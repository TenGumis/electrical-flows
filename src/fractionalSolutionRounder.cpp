#include "fractionalSolutionRounder.h"

#include "matchingGraph.h"
#include "perfectMatchingFinder.h"
#include "randomnessProvider.h"

IntegralFlow FractionalSolutionRounder::roundFlow(const Graph& directedGraph, Flow& flow, unsigned long flowValue)
{
  auto matchingGraph = MatchingGraph::toMatchingGraph(directedGraph, flow, flowValue);
  matchingGraph.toNonPerfectMatching();
  matchingGraph.toPerfectMatching();
  RandomnessProvider randomnessProvider;
  PerfectMatchingFinder perfectMatchingFinder(randomnessProvider);
  perfectMatchingFinder.find(matchingGraph);

  IntegralFlow integralFlow(directedGraph.edges.size());
  for (const auto& edge : directedGraph.edges)
  {
    auto excess = static_cast<int>(flow.getFlow(edge.get()));
    integralFlow.setFlow(edge.get(), excess + static_cast<int>(edge->matchingEquivalent->matched));
  }
  return integralFlow;
}
