#ifndef _FLOW_H_
#define _FLOW_H_

#include "graph.h"
#include <vector>

class Flow
{
public:
    std::vector<double> v;
    
    Flow(int size);

    void update(const Graph& graph, double stepSize, const std::vector<double>& potentials, const std::vector<double>& resistances);
};

#endif // _FLOW_H_