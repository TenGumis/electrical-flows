#include "electricalFlow.h"
#include <iostream>

ElectricalFlow::ElectricalFlow(const Graph& residualGraph):
    resistances(residualGraph.edges.size(), 0.0)
{
    std::cerr << "size: " << residualGraph.edges.size() << std::endl;

    for(int i = 0; i < residualGraph.edges.size(); i++)
    {
        auto forwardCapacity = residualGraph.edges[i]->forwardCapacity;
        auto backwardCapacity = residualGraph.edges[i]->backwardCapacity;
        std::cerr << i << ": " << forwardCapacity << " " << backwardCapacity << std::endl;
        if (forwardCapacity != 0)
        {
            resistances[i] += (1/(forwardCapacity * forwardCapacity));
        }
        if (backwardCapacity != 0 )
        {
            resistances[i] = (1/(forwardCapacity * forwardCapacity));
        }
    }
}