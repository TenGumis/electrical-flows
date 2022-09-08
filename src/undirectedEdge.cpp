#include "undirectedEdge.h"

#include <utility>

UndirectedEdge::UndirectedEdge(std::pair<UndirectedNode*, UndirectedNode*> endpoints,
                               int capacity,
                               int id,
                               bool isPreconditioned)
        : endpoints(std::move(endpoints)),
          capacity(capacity),
          id(id),
          isPreconditioned(isPreconditioned),
          directedEquivalent(nullptr)
{
}
