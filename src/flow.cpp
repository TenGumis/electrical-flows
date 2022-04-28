#include "flow.h"

Flow::Flow(int size):
    v(size)
{

}


void Flow::update(const Graph& graph, double stepSize, const std::vector<double>& potentials, const std::vector<double>& resistances)
{
    for(auto edge : graph.edges)
    {
        double inducedFlow = (potentials[edge->to->label] - potentials[edge->from->label]) / resistances[edge->id];
        v[edge->id] += stepSize * inducedFlow; 
    }
}