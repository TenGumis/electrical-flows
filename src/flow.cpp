#include "flow.h"

Flow::Flow(int size)
        : v(size)
{
}

void Flow::update(const Graph& graph,
                  double stepSize,
                  const std::vector<double>& potentials,
                  const std::vector<double>& resistances)
{
  for (auto edge : graph.edges)
  {
    double inducedFlow = (potentials[edge->to->label] - potentials[edge->from->label]) / resistances[edge->id];
    v[edge->id] += stepSize * inducedFlow;
  }
}

void Flow::correction(const Graph& graph, const std::vector<double>& corrections)
{
  for (auto edge : graph.edges)
  {
    v[edge->id] += corrections[edge->id];
  }
}

double Flow::getEnergy(const std::vector<double>& resistances)
{
  double energy = 0.0;
  for (int i = 0; i < v.size(); i++)
  {
    energy += resistances[i] * v[i] * v[i];
  }

  return energy;
}
