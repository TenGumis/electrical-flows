#include "undirectedNode.h"

UndirectedNode::UndirectedNode(int label)
        : label(label)
{
}

unsigned int UndirectedNode::getDeletedEdgesNumber() const
{
  return deletedEdgesNumber;
}

void UndirectedNode::deleteEdge(int idx)
{
  if (idx < incident.size() - deletedEdgesNumber)
  {
    std::swap(incident[idx], incident[incident.size() - 1 - deletedEdgesNumber]);
    deletedEdgesNumber++;
  }
}
