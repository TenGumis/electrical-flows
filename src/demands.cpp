#include "demands.h"

Demands::Demands(const Graph& graph, double flowValue)
        : demands(graph.nodes.size())
{
  demands[graph.s->label] = -flowValue;
  demands[graph.t->label] = flowValue;
}

Demands::Demands(const Graph& graph, std::vector<double>& corrections)
        : demands(graph.nodes.size())
{
  for (int i = 0; i < graph.nodes.size(); i++)
  {
    for (auto edge : graph.nodes[i]->incoming)
    {
      demands[i] += corrections[edge->id];
    }
    for (auto edge : graph.nodes[i]->outgoing)
    {
      demands[i] -= corrections[edge->id];
    }
  }
}

double Demands::getDemand(Node* node) const
{
  return demands[node->label];
}
