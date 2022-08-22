#include "edge.h"

#include "node.h"

Edge::Edge(Node* from, Node* to, int capacity, int id)
        : id(id),
          from(from),
          to(to),
          capacity(capacity)
{
}
