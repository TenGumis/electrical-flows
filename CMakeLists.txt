cmake_minimum_required(VERSION 3.16)
project(electrical_flows)

set(CMAKE_CXX_STANDARD 20)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCES
    src/main.cpp
    src/node.cpp
    src/graph.cpp
    src/edge.cpp
    src/graph.h
    src/node.h
    src/edge.h
    src/laplacianMatrix.h
    src/laplacianMatrix.cpp
    src/electricalFlow.h
    src/electricalFlow.cpp
    src/flow.h
    src/flow.cpp
    src/embedding.h
    src/embedding.cpp
    src/residualGraph.h
    src/residualGraph.cpp
    src/demands.h
    src/demands.cpp
    src/undirectedNode.h
    src/undirectedNode.cpp
    src/undirectedEdge.h
    src/undirectedEdge.cpp
    src/undirectedGraph.h
    src/undirectedGraph.cpp
    src/helpers.hpp
    src/maxFlowResult.hpp
    src/maxFlowSolver.h
    src/maxFlowSolver.cpp
    src/correctionFlow.h
    src/correctionFlow.cpp
    src/dynamicTrees.h
    src/dynamicTrees.cpp
    src/dynamicTreeNode.h
    src/dynamicTreeNode.cpp
	src/community-code/linkcuttree.h
	src/community-code/linkcuttree.cpp
	src/community-code/splaytree.h
	src/community-code/splaytree.cpp
    src/matchingEdge.cpp
	src/matchingEdge.h
	src/matchingGraph.cpp
	src/matchingGraph.h
	src/matchingNode.cpp
	src/matchingNode.h
	src/perfectMatchingFinder.cpp
	src/perfectMatchingFinder.h
	src/randomnessProvider.cpp
	src/randomnessProvider.h
    src/integralFlow.h
    src/integralFlow.cpp
    src/fractionalSolutionRounder.h
    src/fractionalSolutionRounder.cpp
)

add_executable(electrical-flows ${SOURCES})
target_link_libraries (electrical-flows Eigen3::Eigen)

enable_testing()

add_subdirectory(src)
#add_subdirectory(tests)
#add_subdirectory(lib/googletest)