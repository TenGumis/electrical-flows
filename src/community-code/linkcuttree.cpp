//
//  linkcuttree.cpp
//  Dinic's Algorithm with Link-Cut-Tree
//
//  Created by Сергей Миллер on 12.10.15.
//  Copyright © 2015 Сергей Миллер. All rights reserved.
//

#include "linkcuttree.h"
#include <fstream>
#include <iostream>


LinkCutTree::LinkCutTree(size_t sizeVert){
    nodes.resize(sizeVert, NNode(0));
    for(size_t i = 0;i < nodes.size(); ++i) {
        new SplayTree(&nodes[i]);
        nodes[i].key = i;
    }
}

LinkCutTree::~LinkCutTree() {
    for(size_t i = 0;i < nodes.size(); ++i) {
        if(nodes[i].parent == nullptr) {
            SplayTree* buff = nodes[i].treePtr;
            //assert(buff->_root = &nodes[i]);
            buff->_root = nullptr;
            delete buff;
        }
    }
}

void LinkCutTree::clearTrees() {
    for(size_t i = 0;i < nodes.size(); ++i) {
        if(nodes[i].parent != nullptr) {
            nodes[i] = NNode(i,0);
            nodes[i].treePtr = new SplayTree(&nodes[i]);
        } else {
            SplayTree* buff = nodes[i].treePtr;
            nodes[i] =  NNode(i,0);
            nodes[i].treePtr = buff;
            //buff->_root = &nodes[i];
        }
    }
}

void LinkCutTree::link(size_t indRoot, size_t indVert) {
    NNode* vertex = &nodes[indVert];
    NNode* treeRoot = &nodes[indRoot];
    treeRoot->link = vertex;
    _expose(treeRoot);
}

void LinkCutTree::cut(size_t indVert) {
    NNode* vertex = &nodes[indVert];
    NNode* parent = getParent(indVert);
    _expose(parent);
    vertex->link = nullptr;
}

NNode* LinkCutTree::findRoot(size_t ind) {
    NNode* vertex = &nodes[ind];
    if(vertex != lastExposed) {
        _expose(vertex);
    }
    return _leftest(_liftUpToRoot(vertex));
}

NNode* LinkCutTree::getParent(size_t ind) {
    NNode* vertex = &nodes[ind];
    if(vertex != lastExposed) {
        _expose(vertex);
    }
    _liftUpToRoot(vertex);
    if(vertex->leftChild)
    {
        return _rightest(vertex->leftChild);
    }
    return vertex->link;
}

NNode* LinkCutTree::_cleanUp(NNode* vertex) {
    NNode* root;
    
    if(vertex->parent) {
        root = _cleanUp(vertex->parent);
    } else {
        root = vertex;
    }
    
    NNode::push(vertex);
    
    return root;
}

inline NNode* LinkCutTree::_liftUpToRoot(NNode* vertex) {
    if(!vertex) {
        return nullptr;
    }
    
    if(!vertex->parent) {
        return vertex;
    }
    
    NNode* root = _cleanUp(vertex);
    root->treePtr->splay(vertex);
    return vertex;
}

NNode* LinkCutTree::_leftest(NNode* root) {
    return root->treePtr->find(0);
}

NNode* LinkCutTree::_rightest(NNode* root) {
    return root->treePtr->find(root->sizeOfSubtree - 1);
}

NNode* LinkCutTree::_cutout(NNode* vertex) {
    _liftUpToRoot(vertex);
    std::pair<SplayTree*, SplayTree*> splitedTrees = SplayTree::split(vertex->treePtr, NNode::getSize(vertex->leftChild) + 1);
    SplayTree* right = splitedTrees.second;
    if(right->getRoot()) {
        right->find(0)->link = vertex;
    } else {
        delete right;
    }
    return vertex;
}

NNode* LinkCutTree::_expose(NNode* vertex) {
    lastExposed = vertex;
    NNode* next;
    vertex = _leftest(_liftUpToRoot(_cutout(vertex)));
    while(vertex->link != nullptr) {
        next = _cutout(vertex->link);
        vertex->link = nullptr;
        SplayTree::merge(_liftUpToRoot(next)->treePtr, _liftUpToRoot(vertex)->treePtr);
        vertex = _leftest(_liftUpToRoot(vertex));
    }
    return vertex;
}

NNode* LinkCutTree::getMinEdge(size_t ind) {
    NNode* vertex = &nodes[ind];
    _liftUpToRoot(vertex);
    auto minValue = NNode::getMin(vertex);
    return _findLeftestMin(minValue, vertex);
}

NNode* LinkCutTree::_findLeftestMin(double minValue, NNode* vertex) {
    NNode::push(vertex);
    
    if(NNode::getMin(vertex->leftChild) == minValue) {
        return _findLeftestMin(minValue, vertex->leftChild);
    }
    
    if(vertex->edgeWeight == minValue) {
        return vertex;
    }
    
    return _findLeftestMin(minValue, vertex->rightChild);
}

void LinkCutTree::setWeight(size_t indVert, double weight) {
    NNode* vertex = &nodes[indVert];
    _liftUpToRoot(vertex);
    vertex->edgeWeight = weight;
    NNode::updateNNodeParams(vertex);
}

void LinkCutTree::removeWeightInPath(double added, size_t indVert) {
    _liftUpToRoot(&nodes[indVert]);
    NNode::removeWeight(added, &nodes[indVert]);
}

double LinkCutTree::getEdgeWeight(size_t indVert) {
    NNode* vertex = &nodes[indVert];
    _liftUpToRoot(vertex);
    NNode::push(vertex);
    auto edgeWeight = vertex->edgeWeight;
    return edgeWeight;
}

NNode* LinkCutTree::prevInPath(size_t ind) {
    NNode* source = &nodes[ind];
    _expose(findRoot(ind));
    return _leftest(_liftUpToRoot(source));
}