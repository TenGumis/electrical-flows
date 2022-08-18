# Works with 3.11 and tested through 3.24
cmake_minimum_required(VERSION 3.16)

# Project name and a few useful settings. Other commands can pick up the results
project(
  rounding-flow
  VERSION 0.1
  DESCRIPTION "Rounding flow project"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

set(SOURCES
  edge.cpp
  edge.h
  matchingEdge.cpp
  matchingEdge.h
  matchingGraph.cpp
  matchingGraph.h
  matchingNode.cpp
  matchingNode.h
  node.cpp
  node.h
  randomnessProvider.h
  randomnessProvider.cpp
  perfectMatchingFinder.h
  perfectMatchingFinder.cpp
  round.cpp
)

add_executable(rounding-flow ${SOURCES})

enable_testing()
