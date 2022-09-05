#include "demands.h"

Demands::Demands(const UndirectedGraph& undirectedGraph, double flowValue)
        : demands(undirectedGraph.nodes.size())
{
  demands[undirectedGraph.source->label] = -flowValue;
  demands[undirectedGraph.target->label] = flowValue;
}

Demands::Demands(const UndirectedGraph& undirectedGraph, const CorrectionFlow& correctionFlow)
        : demands(undirectedGraph.nodes.size())
{
  for (const auto& node : undirectedGraph.nodes)
  {
    for (auto edge : node->incident)
    {
      demands[node->label] += (edge->endpoints.first == node.get()) ? correctionFlow.getCorrectionFlow(edge)
                                                                    : -correctionFlow.getCorrectionFlow(edge);
    }
  }
}

double Demands::getDemand(const std::shared_ptr<UndirectedNode>& node) const
{
  return demands[node->label];
}
