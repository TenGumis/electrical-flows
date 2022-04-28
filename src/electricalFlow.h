#ifndef _ELECTRICAL_FLOW_H_
#define _ELECTRICAL_FLOW_H_

#include "graph.h"
#include "laplacianMatrix.h"
#include <vector>

class ElectricalFlow
{
public:
    std::vector<double> resistances;
    const Graph& residualGraph;

    ElectricalFlow(const Graph& residualGraph);

    std::vector<double> computePotentials(const std::vector<double>& demandings, double flowValue);
};

#endif // _ELECTRICAL_FLOW_H_