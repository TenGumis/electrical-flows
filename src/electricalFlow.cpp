#include "electricalFlow.h"

#include <Eigen/Dense>
#include <iostream>

ElectricalFlow::ElectricalFlow(const ResidualGraph& residualGraph)
        : resistances(residualGraph.getNumberOfEdges(), 0.0)
{
  std::cerr << "size: " << residualGraph.getNumberOfEdges() << std::endl;

  for (int i = 0; i < residualGraph.getNumberOfEdges(); i++)
  {
    auto forwardCapacity = residualGraph.getForwardCapacity(i);
    auto backwardCapacity = residualGraph.getBackwardCapacity(i);
    std::cerr << i << ": " << forwardCapacity << " " << backwardCapacity << std::endl;
    if (forwardCapacity != 0)
    {
      resistances[i] += (1 / (forwardCapacity * forwardCapacity));
    }
    if (backwardCapacity != 0)
    {
      resistances[i] += (1 / (backwardCapacity * backwardCapacity));
    }
  }

  std::cerr << "resists:" << std::endl;
  for (auto r : resistances)
  {
    std::cerr << r << " ";
  }
  std::cerr << std::endl;
}

std::vector<double> ElectricalFlow::solveLinearSystemEigen(const Graph& graph,
                                                           const LaplacianMatrix& laplacianMatrix,
                                                           const Demands& demands)
{
  int size = laplacianMatrix.size;

  Eigen::MatrixXd m(size, size);
  m(0, 0) = 3;
  m(1, 0) = 2.5;
  m(0, 1) = -1;
  m(1, 1) = m(1, 0) + m(0, 1);
  for (int row = 0; row < size; ++row)
  {
    for (int col = 0; col < size; ++col)
    {
      m(row, col) = laplacianMatrix.v[row][col];
    }
  }
  std::cerr << "matrix\n" << m << std::endl;

  Eigen::VectorXd b(size);
  for (int y = 0; y < size; ++y)
  {
    b(y) = demands.getDemand(graph.nodes[y].get());
  }
  std::cerr << "demands\n" << b << std::endl;

  Eigen::VectorXd result = m.fullPivLu().solve(b);

  std::cerr << "The solution is:\n" << result << std::endl;

  std::vector<double> tmp(result.data(), result.data() + result.size());

  return tmp;
}

std::vector<double> ElectricalFlow::computePotentials(const Graph& graph, const Demands& demands)
{
  LaplacianMatrix laplacianMatrix(graph, resistances);

  /*
  std::cerr << "Laplacian: " << std::endl;
  for(auto row : laplacianMatrix.v)
  {
      for(auto elem : row)
      {
          std::cerr << elem << " ";
      }
      std::cerr << std::endl;
  }
  std::cerr << std::endl;
  */

  // return solveLinearSystem(laplacianMatrix, demands);
  return solveLinearSystemEigen(graph, laplacianMatrix, demands);
}