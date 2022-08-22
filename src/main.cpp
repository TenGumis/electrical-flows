#include "demands.h"
#include "embedding.h"
#include "flow.h"
#include "graph.h"
#include "laplacianMatrix.h"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

double geAbsoluteStepSize(const double primalProgress, const double m)
{
  double C_eps = 200;

  return (1 - primalProgress) / (33 * sqrt(C_eps * m));
}

double getRelativeProgressStep(const double m)
{
  double C_eps = 200;

  return 1 / (33 * sqrt(C_eps * m));
}

Graph getInput(std::istream& inputStream)
{
  Graph graph;
  int numberOfNodes, numberOfEdges;
  inputStream >> numberOfNodes >> numberOfEdges;

  for (int i = 0; i < numberOfNodes; i++)
  {
    Node newNode(i);

    graph.addNode(std::make_shared<Node>(newNode));
  }

  for (int i = 0; i < numberOfEdges; i++)
  {
    int from, to, capacity;
    inputStream >> from >> to >> capacity;
    auto nodeFrom = graph.nodes[from].get();
    auto nodeTo = graph.nodes[to].get();
    auto newEdge = std::make_shared<Edge>(nodeFrom, nodeTo, capacity, i);

    graph.addEdge(newEdge);
    nodeFrom->outgoing.push_back(newEdge.get());
    nodeTo->incoming.push_back(newEdge.get());
  }

  int s, t;
  inputStream >> s >> t;
  graph.s = graph.nodes[s].get();
  graph.t = graph.nodes[t].get();

  return graph;
}

bool gammaCouplingCheck(const ResidualGraph& residualGraph, const Embedding& embedding, std::vector<double>& violation)
{
  std::cerr << "gammaCouplingCheck" << std::endl;
  for (int i = 0; i < residualGraph.getNumberOfEdges(); i++)
  {
    auto forwardCapacity = residualGraph.getForwardCapcity(i);
    auto backwardCapacity = residualGraph.getBackwardCapcity(i);
    double potential = 0.0;

    if (forwardCapacity != 0)
    {
      potential += 1 / forwardCapacity;
    }
    if (backwardCapacity != 0)
    {
      potential -= 1 / backwardCapacity;
    }

    auto edge = residualGraph.getEdge(i);
    double stretch = embedding.v[edge->to->label] - embedding.v[edge->from->label];

    std::cerr << forwardCapacity << " " << backwardCapacity << std::endl;
    assert(std::min(forwardCapacity, backwardCapacity) != 0.0);

    if (stretch - potential > violation[edge->id] / std::min(forwardCapacity, backwardCapacity))
    {
      assert(false);
      return false;
    }
  }

  return true;
}

std::vector<double> getCorrections(const ResidualGraph& residualGraph, const Embedding& embedding)
{
  std::vector<double> corrections(residualGraph.getNumberOfEdges(), 0.0);

  for (int i = 0; i < residualGraph.getNumberOfEdges(); i++)
  {
    auto forwardCapacity = residualGraph.getForwardCapcity(i);
    auto backwardCapacity = residualGraph.getBackwardCapcity(i);
    double c = 0.0;
    double potential = 0.0;

    if (forwardCapacity != 0)
    {
      c += (1 / (forwardCapacity * forwardCapacity));
      potential += 1 / forwardCapacity;
    }
    if (backwardCapacity != 0)
    {
      c += (1 / (backwardCapacity * backwardCapacity));
      potential -= 1 / backwardCapacity;
    }

    auto edge = residualGraph.getEdge(i);
    double stretch = embedding.v[edge->to->label] - embedding.v[edge->from->label];

    std::cerr << forwardCapacity << " " << backwardCapacity << std::endl;
    assert(c != 0.0);

    corrections[edge->id] = (stretch - potential) / c;
  }

  return corrections;
}

template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

void capacityConstraintCheck(const Graph& graph, const Flow& flow)
{
  for (auto edge : graph.edges)
  {
    assert(flow.v[edge->id] <= edge->capacity);
    assert(flow.v[edge->id] >= -edge->capacity);
  }
  // -ue- <= fe <= ue+;
}

void flowConservationConstraintsCheck(const Graph& graph, const Flow& flow, const Demands& demands)
{
  for (auto node : graph.nodes)
  {
    double enteringFlow = 0.0;
    for (auto edge : node->incoming)
    {
      enteringFlow += flow.v[edge->id];
    }
    double leavingFlow = 0.0;
    for (auto edge : node->outgoing)
    {
      leavingFlow += flow.v[edge->id];
    }
    assert(almost_equal(enteringFlow - leavingFlow, demands.getDemand(node.get()), 10));
  }
}

void feasibilityCheck(const Graph& graph, const Flow& flow, const Demands& demands)
{
  // flowConservationConstraintsCheck(graph, flow, demands);
  capacityConstraintCheck(graph, flow);
}

double getPrimalProgress(const Graph& graph, const Flow& flow, double flowValue)
{
  double xxx = 0;
  for (auto edge : graph.s->outgoing)
  {
    xxx += flow.v[edge->id];
  }
  for (auto edge : graph.s->incoming)
  {
    xxx -= flow.v[edge->id];
  }

  return xxx / flowValue;
}

void solution()
{
  auto graph = getInput(std::cin);

  double flowValue = 4;
  Demands demands(graph, flowValue);
  std::cerr << "demands: " << std::endl;
  for (auto node : graph.nodes)
  {
    std::cerr << demands.getDemand(node.get()) << " ";
  }
  std::cerr << std::endl;
  double primalProgress = 0.0;
  double dualProgress = 0.0;
  Flow flow(graph.edges.size());
  Embedding embedding(graph.nodes.size());
  ResidualGraph residualGraph(graph, flow);

  // int tmp = 20000;  // flowValue/stepSize;
  int steps = 0;
  while (!almost_equal(primalProgress, 1.0, 10) && primalProgress < 1.0)
  {
    steps++;
    // augumenting
    {
      // gammaCouplingCheck(graph, embedding, );

      std::cerr << "-------------------------------------------------------" << std::endl;

      primalProgress = getPrimalProgress(graph, flow, flowValue);
      // dualProgress = sigma * y;
      std::cerr << "primalProgress: " << primalProgress << std::endl;
      std::cerr << "dualProgress: " << dualProgress << std::endl;
      std::cerr << "steps: " << steps << std::endl;

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto potentials = electricalFlow.computePotentials(graph, demands);
      std::cerr << "stepSize: " << geAbsoluteStepSize(primalProgress, graph.edges.size()) << std::endl;
      std::cerr << "potentials: " << std::endl;
      for (auto elem : potentials)
      {
        std::cerr << elem << " ";
      }
      std::cerr << std::endl;

      double stepSize = geAbsoluteStepSize(primalProgress, graph.edges.size());
      flow.update(graph, stepSize, potentials, electricalFlow.resistances);
      embedding.update(graph, stepSize, potentials);

      std::cerr << "flow: " << std::endl;
      for (auto elem : flow.v)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << elem << " ";
        std::cerr << std::setprecision(ss);
      }
      std::cerr << std::endl;
      std::cerr << "embedding: " << std::endl;
      for (auto elem : embedding.v)
      {
        std::cerr << elem << " ";
      }
      std::cerr << std::endl;

      std::cerr << "primalProgress: " << getPrimalProgress(graph, flow, flowValue) << std::endl;
    }

    // fixing
    {
      // gammaCouplingCheck(graph, embedding, );

      auto corrections = getCorrections(residualGraph, embedding);
      std::cerr << "corrections: " << std::endl;
      for (auto elem : corrections)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << elem << " ";
        std::cerr << std::setprecision(ss);
      }
      std::cerr << std::endl;
      flow.correction(graph, corrections);

      auto electricalFlow = ElectricalFlow(residualGraph);
      auto fixingDemands = Demands(graph, corrections);
      std::cerr << "fixingDemands: " << std::endl;
      for (auto node : graph.nodes)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << fixingDemands.getDemand(node.get())
                  << " ";
        std::cerr << std::setprecision(ss);
      }
      auto potentials = electricalFlow.computePotentials(graph, fixingDemands);

      flow.update(graph, 1.0, potentials, electricalFlow.resistances);
      embedding.update(graph, 1.0, potentials);

      std::cerr << "flow: " << std::endl;
      for (auto elem : flow.v)
      {
        std::streamsize ss = std::cerr.precision();
        std::cerr << std::setprecision(std::numeric_limits<double>::digits10 + 1) << elem << " ";
        std::cerr << std::setprecision(ss);
      }
      std::cerr << std::endl;
    }
    feasibilityCheck(graph, flow, demands);

    getchar();
  }
}

int main()
{
  int z;
  std::cin >> z;
  while (z--)
  {
    solution();
  }
}