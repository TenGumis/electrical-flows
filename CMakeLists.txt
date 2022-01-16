cmake_minimum_required(VERSION 3.16)
project(electrical_flows)

set(CMAKE_CXX_STANDARD 20)

set(SOURCES
    src/main.cpp
    src/node.cpp
    src/graph.cpp
    src/edge.cpp
    src/graph.h
    src/node.h
    src/edge.h
    src/matchingEdge.h
    src/matchingEdge.cpp
    src/fancyNode.h
    src/fancyNode.cpp
    src/matchingGraph.h
    src/matchingGraph.cpp
    src/minCostFlowGraph.h
    src/minCostFlowGraph.cpp
    src/flowNode.h
    src/flowNode.cpp
    src/flowEdge.h
    src/flowEdge.cpp
    src/laplacianMatrix.h
    src/laplacianMatrix.cpp
    src/electricalFlow.h
    src/electricalFlow.cpp
    src/flow.h
    src/flow.cpp
    src/ext/utils.hpp
    src/ext/utils.cpp
    src/ext/matrix.hpp
    src/ext/matrix.cpp
    src/ext/system_solver.hpp
    src/ext/system_solver.cpp
)

add_executable(electrical-flows ${SOURCES})

enable_testing()

add_subdirectory(src)
#add_subdirectory(tests)
#add_subdirectory(lib/googletest)