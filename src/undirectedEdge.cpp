#include "undirectedEdge.h"

UndirectedEdge::UndirectedEdge(std::pair<UndirectedNode*, UndirectedNode*> endpoints, int capacity, int id)
        : endpoints(endpoints),
          capacity(capacity),
          id(id)
{
}
