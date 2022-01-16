#include "laplacianMatrix.h"
#include <cassert>

LaplacianMatrix LaplacianMatrix::toLaplacianMatrix(const Graph& graph, const ElectricalFlow& electricalFlow)
{
    LaplacianMatrix laplacianMatrix(graph.nodes.size());
    assert(graph.edges.size() == electricalFlow.resistances.size());
    
    for(const auto edge : graph.edges)
    {
        auto u = edge->from->label;
        auto v = edge->to->label;
        laplacianMatrix.v[u][v] = -1 / electricalFlow.resistances[edge->id];
    }

    for(int i = 0; i < graph.nodes.size(); i++)
    {
        laplacianMatrix.v[i][i] =  0.0;
        for(const auto edge : graph.nodes[i]->incoming)
        {
            laplacianMatrix.v[i][i] += 1/electricalFlow.resistances[edge->id];
        }
        for(const auto edge : graph.nodes[i]->outgoing)
        {
            laplacianMatrix.v[i][i] += 1/electricalFlow.resistances[edge->id];
        }
    }

    return laplacianMatrix;
}

LaplacianMatrix::LaplacianMatrix(const unsigned int size):
    size(size),
    v(size, std::vector<double>(size, 0.0))
{

}
