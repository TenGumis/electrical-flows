#ifndef _ELECTRICAL_FLOW_H_
#define _ELECTRICAL_FLOW_H_

#include "demands.h"
#include "laplacianMatrix.h"
#include "residualGraph.h"

#include <vector>

class LaplacianMatrix;

class ElectricalFlow
{
 public:
  std::vector<double> resistances;

  ElectricalFlow(const ResidualGraph& residualGraph);

  std::vector<double> computePotentials(const UndirectedGraph& undirectedGraph, const Demands& demands) const;

 private:
  std::vector<double> solveLinearSystem(const LaplacianMatrix& laplacianMatrix, const Demands& demands);
  static std::vector<double> solveLinearSystemEigen(const UndirectedGraph& undirectedGraph,
                                                    const LaplacianMatrix& laplacianMatrix,
                                                    const Demands& demands);
};

#endif  // _ELECTRICAL_FLOW_H_