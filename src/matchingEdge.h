#ifndef _MATCHING_EDGE_H_
#define _MATCHING_EDGE_H_

#include "fancyNode.h"
 
class FancyNode;

class MatchingEdge
{
public:
    const FancyNode* const from;
    const FancyNode* const to;

    MatchingEdge(FancyNode* from, FancyNode* to);
};

#endif // _MATCHING_EDGE_H_