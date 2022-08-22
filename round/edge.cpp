#include "edge.h"

#include "node.h"

Edge::Edge(Node* from, Node* to, double capacity, double flow)
        : from(from),
          to(to),
          capacity(capacity),
          flow(flow)
{
}
