#ifndef _ELECTRICAL_FLOW_H_
#define _ELECTRICAL_FLOW_H_

#include "graph.h"
#include <vector>

class ElectricalFlow
{
public:
    std::vector<double> resistances;

    ElectricalFlow(const Graph& residualGraph);
};

#endif // _ELECTRICAL_FLOW_H_