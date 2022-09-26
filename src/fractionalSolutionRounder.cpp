#include "fractionalSolutionRounder.h"

#include "matchingGraph.h"
#include "perfectMatchingFinder.h"
#include "randomnessProvider.h"

IntegralFlow FractionalSolutionRounder::roundFlow(const Graph& directedGraph, Flow& flow, unsigned long flowValue)
{
  auto matchingGraph = MatchingGraph::toMatchingGraph(directedGraph, flow, flowValue);
  // std::cerr << "fractional b-matching graph created" << std::endl;
  // printGraph(matchingGraph);

  matchingGraph.toNonPerfectMatching();
  // std::cerr << "non-perfect matching graph created" << std::endl;
  // printGraph(matchingGraph);

  matchingGraph.toPerfectMatching();
  // std::cerr << "perfect matching graph created" << std::endl;
  // printGraph(matchingGraph);

  RandomnessProvider randomnessProvider;
  PerfectMatchingFinder perfectMatchingFinder(randomnessProvider);
  perfectMatchingFinder.find(matchingGraph);

  IntegralFlow integralFlow(directedGraph.edges.size());
  for (const auto& edge : directedGraph.edges)
  {
    auto excess = static_cast<int>(flow.getFlow(edge.get()));
    integralFlow.setFlow(edge.get(), excess + static_cast<int>(edge->matchingEquivalent->matched));
  }
  // std::cerr << "rounded:" << std::endl;

  // printFlow(directedGraph, integralFlow);
  return integralFlow;
}
