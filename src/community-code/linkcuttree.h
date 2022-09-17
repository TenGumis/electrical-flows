//
//  linkcuttree.h
//  Dinic's Algorithm with Link-Cut-Tree
//
//  Created by Сергей Миллер on 12.10.15.
//  Copyright © 2015 Сергей Миллер. All rights reserved.
//

#ifndef linkcuttree_h
#define linkcuttree_h

#include <stdio.h>
#include <vector>
#include "splaytree.h"
#include <map>


class LinkCutTree {
    friend class LinkCutBlockFlowFinder;
private:
    std::vector <NNode> nodes;
    
    NNode* _cutout(NNode* vertex);
    NNode* _leftest(NNode* vertex);
    NNode* _rightest(NNode* vertex);
    NNode* _expose(NNode* vertex);
    NNode* _cleanUp(NNode* vertex);
    NNode* _liftUpToRoot(NNode* vertex);   //it's splay current vertex
    NNode* _findLeftestMin(double minValue, NNode* vertex);
public:
    NNode* lastExposed;
    LinkCutTree(size_t _size);
    ~LinkCutTree();
    
    void clearTrees();
    
    void removeWeightInPath(double weight, size_t ind);
    void link(size_t indRoot, size_t indVert);
    void cut(size_t indVert);
    void setWeight(size_t indVert, double weight);

    double getEdgeWeight(size_t indVert);
    NNode* prevInPath(size_t ind);
    NNode* getMinEdge(size_t ind);
    NNode* findRoot(size_t ind);
    NNode* getParent(size_t ind);
};


#endif /* linkcuttree_h */