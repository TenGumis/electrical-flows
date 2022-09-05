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
    src/violation.h
    src/violation.cpp
    src/helpers.hpp
    src/maxFlowResult.hpp
    src/maxFlowSolver.h
    src/maxFlowSolver.cpp
    src/correctionFlow.h
    src/correctionFlow.cpp
)

add_executable(electrical-flows ${SOURCES})
target_link_libraries (electrical-flows Eigen3::Eigen)

enable_testing()

add_subdirectory(src)
#add_subdirectory(tests)
#add_subdirectory(lib/googletest)