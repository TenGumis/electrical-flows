#ifndef _LAPLACIAN_MATRIX_H_
#define _LAPLACIAN_MATRIX_H_

#include "electricalFlow.h"
#include "graph.h"

class LaplacianMatrix
{
 public:
  const unsigned int size;
  std::vector<std::vector<double>> matrix;

  LaplacianMatrix() = default;
  LaplacianMatrix(const UndirectedGraph& undirectedGraph, const std::vector<double>& electricalFlow);
};

#endif  // _LAPLACIAN_MATRIX_H_