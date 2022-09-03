#include "undirectedEdge.h"

#include <utility>

UndirectedEdge::UndirectedEdge(std::pair<UndirectedNode*, UndirectedNode*> endpoints, int capacity, int id)
        : endpoints(std::move(endpoints)),
          capacity(capacity),
          id(id)
{
}
