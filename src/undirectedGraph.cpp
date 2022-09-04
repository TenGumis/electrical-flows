#include "undirectedGraph.h"

#include <algorithm>

void UndirectedGraph::addNode(const std::shared_ptr<UndirectedNode>& newNode)
{
  this->nodes.push_back(newNode);
}

void UndirectedGraph::addEdge(const std::shared_ptr<UndirectedEdge>& newEdge)
{
  this->edges.push_back(newEdge);
  newEdge->endpoints.first->incident.push_back(newEdge.get());
  newEdge->endpoints.second->incident.push_back(newEdge.get());
}

UndirectedGraph UndirectedGraph::fromDirected(const Graph& directedGraph)
{
  UndirectedGraph undirectedGraph;

  std::vector<UndirectedEdge*> sourceEdges(directedGraph.nodes.size());
  std::vector<UndirectedEdge*> targetEdges(directedGraph.nodes.size());

  for (const auto& node : directedGraph.nodes)
  {
    std::shared_ptr<UndirectedNode> newNode = std::make_shared<UndirectedNode>(node->label);
    undirectedGraph.addNode(newNode);
    node->undirectedEquivalent = newNode.get();
  }

  undirectedGraph.source = directedGraph.s->undirectedEquivalent;
  undirectedGraph.target = directedGraph.t->undirectedEquivalent;

  int edgeIdCounter = 0;
  for (const auto& edge : directedGraph.edges)
  {
    std::pair<UndirectedNode*, UndirectedNode*> endpoints{edge->from->undirectedEquivalent,
                                                          edge->to->undirectedEquivalent};
    if (endpoints.first == undirectedGraph.source)
    {
      updateContractedEdge(undirectedGraph,
                           sourceEdges,
                           edge->to->undirectedEquivalent->label,
                           endpoints,
                           edge->capacity,
                           edgeIdCounter);
    }
    else if (endpoints.second == undirectedGraph.target)
    {
      updateContractedEdge(undirectedGraph,
                           targetEdges,
                           edge->from->undirectedEquivalent->label,
                           endpoints,
                           edge->capacity,
                           edgeIdCounter);
    }
    else
    {
      auto newEdge = std::make_shared<UndirectedEdge>(endpoints, edge->capacity, edgeIdCounter++);
      undirectedGraph.addEdge(newEdge);
    }

    updateContractedEdge(undirectedGraph,
                         sourceEdges,
                         endpoints.second->label,
                         {undirectedGraph.source, endpoints.second},
                         edge->capacity,
                         edgeIdCounter);
    if (endpoints.first != undirectedGraph.source)
    {
      updateContractedEdge(undirectedGraph,
                           targetEdges,
                           endpoints.first->label,
                           {endpoints.first, undirectedGraph.target},
                           edge->capacity,
                           edgeIdCounter);
    }
    else
    {
      updateContractedEdge(undirectedGraph,
                           sourceEdges,
                           undirectedGraph.target->label,
                           {undirectedGraph.source, undirectedGraph.target},
                           edge->capacity,
                           edgeIdCounter);
    }
  }

  return std::move(undirectedGraph);
}

void UndirectedGraph::addPreconditioningEdges()
{
  int newId = edges.size();
  int maxCapacity = getMaxCapacity();
  int capacity = 2 * maxCapacity;
  std::pair<UndirectedNode*, UndirectedNode*> endpoints{source, target};

  int numberOfPreconditionEdges = edges.size();
  for (int i = 0; i < numberOfPreconditionEdges; i++)
  {
    auto newEdge = std::make_shared<UndirectedEdge>(endpoints, capacity, newId++);

    addEdge(newEdge);
  }
}

void UndirectedGraph::updateContractedEdge(UndirectedGraph& undirectedGraph,
                                           std::vector<UndirectedEdge*>& contractedEdges,
                                           int label,
                                           std::pair<UndirectedNode*, UndirectedNode*> endpoints,
                                           int capacity,
                                           int& id)
{
  if (contractedEdges[label] == nullptr)
  {
    auto newEdge = std::make_shared<UndirectedEdge>(endpoints, capacity, id++);
    contractedEdges[label] = newEdge.get();
    undirectedGraph.addEdge(newEdge);
  }
  else
  {
    contractedEdges[label]->capacity += capacity;
  }
}

int UndirectedGraph::getMaxCapacity() const
{
  int maxCapacity = 0;
  for (const auto& edge : edges)
  {
    maxCapacity = std::max(maxCapacity, edge->capacity);
  }
  return maxCapacity;
}