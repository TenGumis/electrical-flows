#include "correctionFlow.h"

#include "residualGraph.h"

#include <cassert>

CorrectionFlow::CorrectionFlow(const ResidualGraph& residualGraph, const Embedding& embedding)
        : correction(residualGraph.getNumberOfEdges())
{
  for (const auto& edge : residualGraph.graph.edges)
  {
    auto forwardCapacity = residualGraph.getForwardCapacity(edge, edge->endpoints.first);
    auto backwardCapacity = residualGraph.getBackwardCapacity(edge, edge->endpoints.first);
    double resistance = 0.0;
    double potential = 0.0;

    if (forwardCapacity != 0)
    {
      resistance += (1 / (forwardCapacity * forwardCapacity));
      potential += 1 / forwardCapacity;
    }
    if (backwardCapacity != 0)
    {
      resistance += (1 / (backwardCapacity * backwardCapacity));
      potential -= 1 / backwardCapacity;
    }
    double stretch = embedding.getStretch(edge);

    assert(resistance != 0.0);

    correction[edge->id] = (stretch - potential) / resistance;
  }
}

double CorrectionFlow::getCorrectionFlow(const std::shared_ptr<UndirectedEdge>& edge) const
{
  return correction[edge->id];
}

double CorrectionFlow::getCorrectionFlow(const UndirectedEdge* const edge) const
{
  return correction[edge->id];
}
