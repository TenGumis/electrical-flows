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

  for (const auto& node : directedGraph.nodes)
  {
    std::shared_ptr<UndirectedNode> newNode = std::make_shared<UndirectedNode>(node->label);
    undirectedGraph.addNode(newNode);
    node->undirectedEquivalent = newNode.get();
  }
  undirectedGraph.edgeMapping.resize(directedGraph.nodes.size());

  undirectedGraph.setSourceAndTarget(directedGraph.s->undirectedEquivalent, directedGraph.t->undirectedEquivalent);
  unsigned int edgeIdCounter = 0;
  for (const auto& edge : directedGraph.edges)
  {
    std::pair<UndirectedNode*, UndirectedNode*> endpoints{edge->from->undirectedEquivalent,
                                                          edge->to->undirectedEquivalent};
    if (undirectedGraph.containsOppositeEquivalent(edge.get()))
    {
      undirectedGraph.edgeMapping[edge->to->label][edge->from->label]->capacity += edge->capacity;
    }
    else
    {
      if (endpoints.first == undirectedGraph.source)
      {
        updateContractedEdge(undirectedGraph,
                             undirectedGraph.sourceEdges,
                             edge->to->undirectedEquivalent->label,
                             endpoints,
                             edge->capacity,
                             edgeIdCounter);
        edge->undirectedEquivalent = undirectedGraph.sourceEdges[edge->to->undirectedEquivalent->label];
        undirectedGraph.sourceEdges[edge->to->undirectedEquivalent->label]->directedEquivalent = edge.get();
      }
      else if (endpoints.second == undirectedGraph.target)
      {
        updateContractedEdge(undirectedGraph,
                             undirectedGraph.targetEdges,
                             edge->from->undirectedEquivalent->label,
                             endpoints,
                             edge->capacity,
                             edgeIdCounter);
        edge->undirectedEquivalent = undirectedGraph.targetEdges[edge->from->undirectedEquivalent->label];
        undirectedGraph.targetEdges[edge->from->undirectedEquivalent->label]->directedEquivalent = edge.get();
      }
      else
      {
        auto newEdge = std::make_shared<UndirectedEdge>(endpoints, edge->capacity, edgeIdCounter++);
        undirectedGraph.addEdge(newEdge);
        edge->undirectedEquivalent = newEdge.get();
        newEdge->directedEquivalent = edge.get();
        undirectedGraph.edgeMapping[endpoints.first->label][endpoints.second->label] = newEdge.get();
      }
    }
    addSourceTargetEdges(undirectedGraph, endpoints, edge->capacity, edgeIdCounter);
  }

  if (undirectedGraph.sourceEdges[undirectedGraph.target->label])
  {
    undirectedGraph.targetEdges[undirectedGraph.source->label] =
            undirectedGraph.sourceEdges[undirectedGraph.target->label];
  }

  return std::move(undirectedGraph);
}

void UndirectedGraph::addPreconditioningEdges()
{
  unsigned int newId = edges.size();
  unsigned int capacity = 2 * getMaxCapacity();
  std::pair<UndirectedNode*, UndirectedNode*> endpoints{source, target};

  unsigned int numberOfPreconditionEdges = edges.size();
  for (int i = 0; i < numberOfPreconditionEdges; i++)
  {
    auto newEdge = std::make_shared<UndirectedEdge>(endpoints, capacity, newId++, true);
    addEdge(newEdge);
  }
}

void UndirectedGraph::removePreconditioningEdges()
{
  unsigned int numberOfPreconditionEdges = edges.size() / 2;

  std::vector<UndirectedEdge*> newIncidentSource;
  for (auto edge : source->incident)
  {
    if (!edge->isPreconditioned)
    {
      newIncidentSource.push_back(edge);
    }
  }
  source->incident = newIncidentSource;

  std::vector<UndirectedEdge*> newIncidentTarget;
  for (auto edge : target->incident)
  {
    if (!edge->isPreconditioned)
    {
      newIncidentTarget.push_back(edge);
    }
  }
  target->incident = newIncidentTarget;

  while (numberOfPreconditionEdges--)
  {
    edges.pop_back();
  }
}

void UndirectedGraph::updateContractedEdge(UndirectedGraph& undirectedGraph,
                                           std::vector<UndirectedEdge*>& contractedEdges,
                                           int label,
                                           std::pair<UndirectedNode*, UndirectedNode*> endpoints,
                                           int capacity,
                                           unsigned int& id)
{
  if (contractedEdges[label] == nullptr)
  {
    auto newEdge = std::make_shared<UndirectedEdge>(endpoints, capacity, id++);
    contractedEdges[label] = newEdge.get();
    undirectedGraph.addEdge(newEdge);
    undirectedGraph.edgeMapping[endpoints.first->label][endpoints.second->label] = newEdge.get();
  }
  else
  {
    contractedEdges[label]->capacity += capacity;
  }
}

void UndirectedGraph::addSourceTargetEdges(UndirectedGraph& undirectedGraph,
                                           std::pair<UndirectedNode*, UndirectedNode*>& endpoints,
                                           unsigned int capacity,
                                           unsigned int& edgeIdCounter)
{
  updateContractedEdge(undirectedGraph,
                       undirectedGraph.sourceEdges,
                       endpoints.second->label,
                       {undirectedGraph.source, endpoints.second},
                       static_cast<int>(capacity),
                       edgeIdCounter);
  if (endpoints.first != undirectedGraph.source)
  {
    updateContractedEdge(undirectedGraph,
                         undirectedGraph.targetEdges,
                         endpoints.first->label,
                         {endpoints.first, undirectedGraph.target},
                         static_cast<int>(capacity),
                         edgeIdCounter);
  }
  else
  {
    updateContractedEdge(undirectedGraph,
                         undirectedGraph.sourceEdges,
                         undirectedGraph.target->label,
                         {undirectedGraph.source, undirectedGraph.target},
                         static_cast<int>(capacity),
                         edgeIdCounter);
  }
}

unsigned long UndirectedGraph::getMaxCapacity() const
{
  unsigned long maxCapacity = 0;
  for (const auto& edge : edges)
  {
    maxCapacity = std::max(maxCapacity, static_cast<unsigned long>(edge->capacity));
  }
  return maxCapacity;
}

void UndirectedGraph::setSourceAndTarget(UndirectedNode* newSource, UndirectedNode* newTarget)
{
  source = newSource;
  target = newTarget;
  sourceEdges.resize(nodes.size());
  targetEdges.resize(nodes.size());
}

bool UndirectedGraph::containsOppositeEquivalent(const Edge* edge) const
{
  return edgeMapping[edge->to->label].find(edge->from->label) != edgeMapping[edge->to->label].end();
}

UndirectedEdge* UndirectedGraph::getMappedEquivalent(const Edge* const edge) const
{
  return edgeMapping[edge->to->label].at(edge->from->label);
}
