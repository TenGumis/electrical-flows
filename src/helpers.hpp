#include "flow.h"
#include "embedding.h"

#include <iomanip>
#include <iostream>
#include <limits>

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

void printEmbedding(const UndirectedGraph& undirectedGraph, const Embedding& embedding)
{
  std::cerr << "embedding: " << std::endl;
  for (const auto& node : undirectedGraph.nodes)
  {
    std::cerr << embedding.getEmbedding(node) << " ";
  }
  std::cerr << std::endl;
}

void printCorrections(const std::vector<double>& corrections)
{
  std::cerr << "corrections: " << std::endl;
  int counter = 0;
  for (auto elem : corrections)
  {
    std::cerr << counter++ << ": ";
    std::streamsize ss = std::cerr.precision();
    std::cerr << std::fixed << std::setprecision(std::numeric_limits<double>::digits10 + 1) << elem << "\n";
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
