set(BINARY ${CMAKE_PROJECT_NAME})

set(SOURCES
    main.cpp
    node.cpp
    graph.cpp
    edge.cpp
    graph.h
    node.h
    edge.h
    matchingEdge.h
    matchingEdge.cpp
    fancyNode.h
    fancyNode.cpp
    matchingGraph.h
    matchingGraph.cpp
    minCostFlowGraph.h
    minCostFlowGraph.cpp
    flowNode.h
    flowNode.cpp
    flowEdge.h
    flowEdge.cpp
    laplacianMatrix.h
    laplacianMatrix.cpp
    electricalFlow.h
    electricalFlow.cpp
    flow.h
    flow.cpp
    ext/utils.hpp
    ext/utils.cpp
    ext/matrix.hpp
    ext/matrix.cpp
    ext/system_solver.hpp
    ext/system_solver.cpp
)

add_library(${BINARY}_lib STATIC ${SOURCES})