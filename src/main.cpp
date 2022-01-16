#include "graph.h"
#include "fancyNode.h"
#include "matchingEdge.h"
#include "matchingGraph.h"
#include "minCostFlowGraph.h"
#include "laplacianMatrix.h"
#include "flow.h"
#include "ext/matrix.hpp"
#include "ext/system_solver.hpp"
#include <iostream>
#include <vector>
#include <cmath>

double getStepSize(const double m)
{
    return 1/(33 * sqrt(200 * m));
}

std::vector<double> getDemandings(const Graph& graph, double flow)
{
    std::vector<double> result(graph.nodes.size());

    result[graph.s->label] = -flow;
    result[graph.t->label] = flow;

    return result;
}

std::vector<double> solveLinearSystem(const LaplacianMatrix& laplacianMatrix, const std::vector<double>& demandings)
{
    int width = laplacianMatrix.size + 1;
    int height = laplacianMatrix.size;

    Matrix matrix(width, height, true);

    for (int y = 0; y < laplacianMatrix.size; ++y) {
        for (int x = 0; x < laplacianMatrix.size; ++x) {
            matrix.set_field(x, y, laplacianMatrix.v[y][x]);
        }
    }

    for (int y = 0; y < height; ++y) {
        matrix.set_field(width - 1, y, demandings[y]);
    }

    return SystemSolver::solve(matrix);
}

Graph getInput(std::istream& inputStream)
{
    Graph graph;
    int numberOfNodes,numberOfEdges;
    inputStream >> numberOfNodes >> numberOfEdges;

    for(int i = 0; i < numberOfNodes; i++)
    {
        Node newNode(i);

        graph.addNode(std::make_shared<Node>(newNode));
    }

    for(int i = 0; i < numberOfEdges; i++)
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

void solution()
{
    auto graph = getInput(std::cin);
    auto matchingGraph = MatchingGraph::toMatchingGraph(graph, 2);

    std::cout << matchingGraph.nodesP.size() << " " << matchingGraph.nodesQ.size() << std::endl;
    std::cout << matchingGraph.edges.size() << std::endl;

    auto minCostFlowGraph = MinCostFlowGraph::toMinCostFlowGraph(matchingGraph);
    std::cout << minCostFlowGraph.nodes.size() << std::endl;
    std::cout << minCostFlowGraph.edges.size() << std::endl;

    std::cerr << "ooeoeoe" << std::endl;
    auto electricalFlow = ElectricalFlow(graph);
    std::cerr << "resists:" << std::endl;
    for(auto r : electricalFlow.resistances)
    {
        std::cerr << r << " "; 
    }
    std::cerr << std::endl;
    auto laplacian = LaplacianMatrix::toLaplacianMatrix(graph, electricalFlow);
    std::cerr << "Laplacian: " << std::endl;
    for(auto row : laplacian.v)
    {
        for(auto elem : row)
        {
            std::cerr << elem << " ";
        }
        std::cerr << std::endl;
    }
    std::cerr << std::endl;
    double flowValue = 4;
    auto demandings = getDemandings(graph, flowValue);
    std::cerr << "demandings: " << std::endl;
    for(auto elem : demandings)
    {
        std::cerr << elem << " ";
    }
    auto potentials = solveLinearSystem(laplacian, demandings);
    double stepSize = getStepSize(graph.edges.size());
    std::cerr << "stepSize: " << stepSize << std::endl;
    std::cerr << "potentials: " << std::endl;
    for(auto elem : potentials)
    {
        std::cerr << elem << " ";
    }
    std::cerr << std::endl;
    Flow flow;
    flow.update(stepSize, potentials);
} 

int main()
{
    int z;
    std::cin >> z;
    while(z--)
    {
        solution();
    }
} 