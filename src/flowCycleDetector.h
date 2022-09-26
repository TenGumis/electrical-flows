#ifndef _FLOW_CYCLE_DETECTOR_H_
#define _FLOW_CYCLE_DETECTOR_H_

#include <vector>

class Graph;
class UndirectedGraph;
class Flow;
class UndirectedNode;
class UndirectedEdge;

class FlowCycleDetector
{
 public:
  static bool containsFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);
  static void removeFlowCycles(const Graph& directedGraph, const UndirectedGraph& undirectedGraph, Flow& flow);

 private:
  static bool hasOutgoingFlow(const UndirectedNode* undirectedNode,
                              const Flow& flow,
                              const std::vector<bool>& deletedEdges);
  static UndirectedEdge* getOutgoingFlowEdge(const UndirectedNode* undirectedNode,
                                             const Flow& flow,
                                             const std::vector<bool>& deletedEdges);
};

#endif  // _FLOW_CYCLE_DETECTOR_H_