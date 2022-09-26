#ifndef _HELPERS_H_
#define _HELPERS_H_

#include "flow.h"
#include "embedding.h"
#include "correctionFlow.h"
#include "matchingGraph.h"
#include "integralFlow.h"

#include <iomanip>
#include <iostream>
#include <limits>
#include <cassert>
#include <cmath>

void printFlow(const UndirectedGraph& undirectedGraph, const Flow& flow)
{
  std::cerr << "flow: " << std::endl;
  int counter = 0;
  for (const auto& edge : undirectedGraph.edges)
  {
    std::cerr << counter++ << ": ";
    auto value = flow.getFlow(edge.get(), edge->endpoints.first);
    std::streamsize ss = std::cerr.precision();
    std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << value << "\n";
    std::cerr << std::setprecision(ss);
  }
  std::cerr << std::endl;
}

void printFlow(const Graph& graph, const Flow& flow)
{
    std::cerr << "flow: " << std::endl;
    int counter = 0;
    for (const auto& edge : graph.edges)
    {
        std::cerr << counter++ << ": ";
        auto value = flow.getFlow(edge.get());
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << value << "\n";
        std::cerr << std::setprecision(ss);
    }
    std::cerr << std::endl;
}

void printFlow(const Graph& graph, const IntegralFlow& flow)
{
    std::cerr << "integral flow: " << std::endl;
    int counter = 0;
    for (const auto& edge : graph.edges)
    {
        std::cerr << counter++ << ": ";
        auto value = flow.getFlow(edge.get());
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << value << "\n";
        std::cerr << std::setprecision(ss);
    }
    std::cerr << std::endl;
}

void printEmbedding(const UndirectedGraph& undirectedGraph, const Embedding& embedding)
{
  std::cerr << "embedding: " << std::endl;
  for (const auto& node : undirectedGraph.nodes)
  {
    std::cerr << embedding.getEmbedding(node) << " ";
  }
  std::cerr << std::endl;
}

void printCorrections(const UndirectedGraph& undirectedGraph, const CorrectionFlow& correctionFlow)
{
  std::cerr << "corrections: " << std::endl;
  int counter = 0;
  for (const auto& edge : undirectedGraph.edges)
  {
    std::cerr << counter++ << ": ";
    std::streamsize ss = std::cerr.precision();
    std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << correctionFlow.getCorrectionFlow(edge) << "\n";
    std::cerr << std::setprecision(ss);
  }
  std::cerr << std::endl;
}

void printInducedFlow(const UndirectedGraph& undirectedGraph, std::vector<double> potentials, const std::vector<double>& resistances)
{
  std::cerr << "inducedFlow: " << std::endl;
  int counter = 0;
  for (const auto& edge : undirectedGraph.edges)
  {
    std::cerr << counter++ << ": ";
    double inducedFlow = (potentials[edge->endpoints.second->label] - potentials[edge->endpoints.first->label]) /
                         resistances[edge->id];
    std::streamsize ss = std::cerr.precision();
    std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << inducedFlow << "\n";
    std::cerr << std::setprecision(ss);
  }
}

void printGraph(const MatchingGraph& matchingGraph)
{
  std::cerr << "nodesP: " << matchingGraph.nodesP.size() << std::endl;
  double totalFlow = 0;
  for (const auto& node : matchingGraph.nodesP)
  {
    double flow = 0;
    for (auto edge : node->edges)
    {
      flow += edge->flow;
      totalFlow += edge->flow;
      assert(edge->pNode->id == node->id);
    }
    std::cerr << node->id << " " << flow << "/" << node->demand << " " << node->edges.size() << std::endl;
  }
  std::cerr << "totalFlow: " << totalFlow << std::endl;

  std::cerr << "nodesQ: " << matchingGraph.nodesQ.size() << std::endl;
  totalFlow = 0;
  for (const auto& node : matchingGraph.nodesQ)
  {
    double flow = 0;
    for (auto edge : node->edges)
    {
      flow += edge->flow;
      totalFlow += edge->flow;
      assert(edge->qNode->id == node->id);
    }
    std::cerr << node->id << " " << flow << "/" << node->demand << " " << node->edges.size() << std::endl;
  }
  std::cerr << "totalFlow: " << totalFlow << std::endl;

  std::cerr << "edges: " << matchingGraph.edges.size() << std::endl;
  for (const auto& edge : matchingGraph.edges)
  {
    std::cerr << edge->pNode->id << " " << edge->qNode->id << " " << edge->flow << std::endl;
  }
}

#endif // _HELPERS_H_