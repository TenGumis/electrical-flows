#ifndef _FLOW_H_
#define _FLOW_H_

#include <vector>

class Flow
{
public:
    void update(double stepSize, const std::vector<double>& potentials);
};

#endif // _FLOW_H_