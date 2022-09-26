#include "electricalFlow.h"

#include "demands.h"

#include <Eigen/Dense>
ElectricalFlow::ElectricalFlow(const ResidualGraph& residualGraph)
        : resistances(residualGraph.getNumberOfEdges(), 0.0)
{
  for (int i = 0; i < residualGraph.getNumberOfEdges(); i++)
  {
    const auto& edge = residualGraph.graph.edges[i];
    auto forwardCapacity = residualGraph.getForwardCapacity(edge, edge->endpoints.first);
    auto backwardCapacity = residualGraph.getBackwardCapacity(edge, edge->endpoints.first);
    if (forwardCapacity != 0)
    {
      resistances[i] += (1 / (forwardCapacity * forwardCapacity));
    }
    if (backwardCapacity != 0)
    {
      resistances[i] += (1 / (backwardCapacity * backwardCapacity));
    }
  }
}

std::vector<double> ElectricalFlow::solveLinearSystemEigen(const UndirectedGraph& undirectedGraph,
                                                           const LaplacianMatrix& laplacianMatrix,
                                                           const Demands& demands)
{
  auto size = laplacianMatrix.size;

  Eigen::MatrixXd m(size, size);
  for (int row = 0; row < size; ++row)
  {
    for (int col = 0; col < size; ++col)
    {
      m(row, col) = laplacianMatrix.matrix[row][col];
    }
  }

  Eigen::VectorXd b(size);
  for (int y = 0; y < size; ++y)
  {
    b(y) = demands.getDemand(undirectedGraph.nodes[y]);
  }

  Eigen::VectorXd result = m.fullPivLu().solve(b);
  std::vector<double> tmp(result.data(), result.data() + result.size());

  return tmp;
}

std::vector<double> ElectricalFlow::computePotentials(const UndirectedGraph& undirectedGraph,
                                                      const Demands& demands) const
{
  LaplacianMatrix laplacianMatrix(undirectedGraph, resistances);
  return solveLinearSystemEigen(undirectedGraph, laplacianMatrix, demands);
}