#include "flow.h"

#include "correctionFlow.h"

Flow::Flow(int size)
        : flow(size)
{
}

void Flow::update(const UndirectedGraph& undirectedGraph,
                  double stepSize,
                  const std::vector<double>& potentials,
                  const std::vector<double>& resistances)
{
  for (const auto& edge : undirectedGraph.edges)
  {
    double inducedFlow = (potentials[edge->endpoints.second->label] - potentials[edge->endpoints.first->label]) /
                         resistances[edge->id];
    flow[edge->id] += stepSize * inducedFlow;
  }
}

double Flow::getFlow(const Edge* const edge) const
{
  return flow[edge->id];
}

double Flow::getFlow(const UndirectedEdge* const edge, const UndirectedNode* const endpoint) const
{
  return (edge->endpoints.first == endpoint) ? flow[edge->id] : -flow[edge->id];
}

void Flow::updateFlow(const UndirectedEdge* const edge, const UndirectedNode* const endpoint, const double value)
{
  if (edge->endpoints.first == endpoint)
  {
    flow[edge->id] += value;
  }
  else
  {
    flow[edge->id] -= value;
  }
}

void Flow::setFlow(const UndirectedEdge* const edge, const UndirectedNode* const endpoint, const double value)
{
  if (edge->endpoints.first == endpoint || value == 0.0)
  {
    flow[edge->id] = value;
  }
  else
  {
    flow[edge->id] = -value;
  }
}

void Flow::setFlow(const Edge* edge, double value)
{
  flow[edge->id] = value;
}

void Flow::applyCorrectionFlow(const UndirectedGraph& undirectedGraph, const CorrectionFlow& correctionFlow)
{
  for (const auto& edge : undirectedGraph.edges)
  {
    flow[edge->id] += correctionFlow.getCorrectionFlow(edge);
  }
}

void Flow::scaleDown()
{
  const double SCALING_FACTOR = 2.0;
  for (auto& elem : flow)
  {
    elem /= SCALING_FACTOR;
  }
}

double Flow::getEnergy(const std::vector<double>& resistances)
{
  double energy = 0.0;
  for (int i = 0; i < flow.size(); i++)
  {
    energy += resistances[i] * flow[i] * flow[i];
  }

  return energy;
}