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

  std::vector<double> computePotentials(const Graph& graph, const Demands& demandings);

 private:
  std::vector<double> solveLinearSystem(const LaplacianMatrix& laplacianMatrix, const Demands& demandings);
  std::vector<double> solveLinearSystemEigen(const Graph& graph,
                                             const LaplacianMatrix& laplacianMatrix,
                                             const Demands& demandings);
};

#endif  // _ELECTRICAL_FLOW_H_