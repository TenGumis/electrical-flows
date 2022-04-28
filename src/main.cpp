#include "graph.h"
#include "fancyNode.h"
#include "matchingEdge.h"
#include "matchingGraph.h"
#include "minCostFlowGraph.h"
#include "laplacianMatrix.h"
#include "flow.h"
#include "embedding.h"
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
    
    double flowValue = 4;
    auto demandings = getDemandings(graph, flowValue);
    std::cerr << "demandings: " << std::endl;
    for(auto elem : demandings)
    {
        std::cerr << elem << " ";
    }
    double stepSize = getStepSize(graph.edges.size());
    Flow flow(graph.edges.size());
    Embedding embedding(graph.nodes.size());
    //input: demandings, 

    int tmp = 1000;
    while(tmp--)
    {

        //augumenting
        std::cerr << "-------------------------------------------------------" << std::endl;
        auto electricalFlow = ElectricalFlow(graph);
        auto potentials = electricalFlow.computePotentials(demandings, flowValue);
        std::cerr << "stepSize: " << stepSize << std::endl;
        std::cerr << "potentials: " << std::endl;
        for(auto elem : potentials)
        {
            std::cerr << elem << " ";
        }
        std::cerr << std::endl;

        flow.update(graph, stepSize, potentials, electricalFlow.resistances);
        embedding.update(graph, stepSize, potentials);
        graph.updateFlow(flow.v);
        
        std::cerr << "flow: " << std::endl;
        for(auto elem : flow.v)
        {
            std::cerr << elem << " ";
        }
        std::cerr << std::endl;
        std::cerr << "embedding: " << std::endl;
        for(auto elem : embedding.v)
        {
            std::cerr << elem << " ";
        }
        std::cerr << std::endl;

        //fixing


        //sprawdzić niezmiennink couplingu
    }
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