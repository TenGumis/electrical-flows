#ifndef _LAPLACIAN_MATRIX_H_
#define _LAPLACIAN_MATRIX_H_

#include "graph.h"
#include "electricalFlow.h"

class LaplacianMatrix
{
public:
    const unsigned int size;
    std::vector<std::vector<double>> v;
 
    static LaplacianMatrix toLaplacianMatrix(const Graph& graph, const ElectricalFlow& electricalFlow);

private:
    LaplacianMatrix(const unsigned int size);
};

#endif // _LAPLACIAN_MATRIX_H_