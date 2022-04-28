#include "electricalFlow.h"
#include "ext/matrix.hpp"
#include "ext/system_solver.hpp"
#include <iostream>

ElectricalFlow::ElectricalFlow(const Graph& residualGraph):
    resistances(residualGraph.edges.size(), 0.0),
    residualGraph(residualGraph)
{
    std::cerr << "size: " << residualGraph.edges.size() << std::endl;

    for(int i = 0; i < residualGraph.edges.size(); i++)
    {
        auto forwardCapacity = residualGraph.edges[i]->forwardCapacity;
        auto backwardCapacity = residualGraph.edges[i]->backwardCapacity;
        std::cerr << i << ": " << forwardCapacity << " " << backwardCapacity << std::endl;
        if (forwardCapacity != 0)
        {
            resistances[i] += (1/(forwardCapacity * forwardCapacity));
        }
        if (backwardCapacity != 0 )
        {
            resistances[i] = (1/(forwardCapacity * forwardCapacity));
        }
    }

    std::cerr << "resists:" << std::endl;
    for(auto r : resistances)
    {
        std::cerr << r << " "; 
    }
    std::cerr << std::endl;

}

std::vector<double> solveLinearSystem(const LaplacianMatrix& laplacianMatrix, const std::vector<double>& demandings)
{
    int width = laplacianMatrix.size + 1;
    int height = laplacianMatrix.size;

    Matrix matrix(width, height, true);

    for (int y = 0; y < laplacianMatrix.size; ++y) {
        for (int x = 0; x < laplacianMatrix.size; ++x) {
            matrix.set_field(x, y, laplacianMatrix.v[y][x]);
        }
    }

    for (int y = 0; y < height; ++y) {
        matrix.set_field(width - 1, y, demandings[y]);
    }

    return SystemSolver::solve(matrix);
}

std::vector<double> ElectricalFlow::computePotentials(const std::vector<double>& demandings, double flowValue)
{
    LaplacianMatrix laplacianMatrix(residualGraph, resistances);

    /*
    std::cerr << "Laplacian: " << std::endl;
    for(auto row : laplacian.v)
    {
        for(auto elem : row)
        {
            std::cerr << elem << " ";
        }
        std::cerr << std::endl;
    }
    std::cerr << std::endl;
    */

    return solveLinearSystem(laplacianMatrix, demandings);
}