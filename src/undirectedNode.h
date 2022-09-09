#ifndef _UNDIRECTED_NODE_H_
#define _UNDIRECTED_NODE_H_

#include "undirectedEdge.h"

#include <list>
#include <memory>
#include <vector>

class UndirectedEdge;

class UndirectedNode
{
 private:
  unsigned int deletedEdgesNumber;

 public:
  const int label;
  std::vector<UndirectedEdge*> incident;

  UndirectedNode(int label);
  unsigned int getDeletedEdgesNumber() const;
  void deleteEdge(int idx);
};

#endif  // _UNDIRECTED_NODE_H_