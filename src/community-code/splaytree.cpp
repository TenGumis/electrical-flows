//
//  splaytree.cpp
//  Dinic's Algorithm with Link-Cut-Tree
//
//  Created by Сергей Миллер on 10.10.15.
//  Copyright © 2015 Сергей Миллер. All rights reserved.
//

#include "splaytree.h"
#include <cstdio>
#include <vector>


NNode::NNode(size_t key, size_t edgeWeight):key(key), sizeOfSubtree(1), leftChild(nullptr), rightChild(nullptr), parent(nullptr), link(nullptr), subtreeMinWeight(edgeWeight), subtreeMaxWeight(edgeWeight), removedWeightValue(0), treePtr(nullptr), edgeWeight(edgeWeight), subtreeWeight(edgeWeight), reverseFlag(false) {
}

void NNode::recursiveDelete(NNode* vertex) {
    if(vertex) {
        NNode::recursiveDelete(vertex->leftChild);
        NNode::recursiveDelete(vertex->rightChild);
        delete vertex;
    }
}

void NNode::removeWeight(double value, NNode* vertex) {
    if(vertex) {
        vertex->removedWeightValue += value;
    }
}

void NNode::reverse(NNode* vertex) {
    if(vertex) {
        vertex->reverseFlag ^= true;
    }
}

void NNode::push(NNode* vertex) {
    if(vertex) {
        vertex->edgeWeight -= vertex->removedWeightValue;
        NNode::removeWeight(vertex->removedWeightValue, vertex->leftChild);
        NNode::removeWeight(vertex->removedWeightValue, vertex->rightChild);
        vertex->removedWeightValue = 0;
        NNode::updateNNodeParams(vertex);
        if(vertex->reverseFlag) { //guarantee that this != NULL
            std::swap(vertex->leftChild, vertex->rightChild);
            vertex->reverseFlag = false;
            
            NNode::reverse(vertex->leftChild);
            NNode::reverse(vertex->rightChild);
        }
    }
}

SplayTree::SplayTree(NNode* root) {
    _root = root;
    if(root) {
        root->treePtr = this;
    }
}

SplayTree::~SplayTree() {
    NNode::recursiveDelete(_root);
}

inline size_t NNode::getSize(NNode* vertex) {
    return (vertex ? vertex->sizeOfSubtree : 0);
}

inline double NNode::getMin(NNode* vertex) {
    NNode::push(vertex);
    return (vertex ? vertex->subtreeMinWeight - vertex->removedWeightValue : INF);
}

inline double NNode::getMax(NNode* vertex) {
    return (vertex ? vertex->subtreeMaxWeight - vertex->removedWeightValue : 0);
}

size_t NNode::getKey(NNode* vertex) {
    return (vertex ? vertex->key : INF);
}

double NNode::getWeight(NNode* vertex) {
    return (vertex ? vertex->edgeWeight : INF);
}

double NNode::getSubtreeWeight(NNode* vertex) {
    return (vertex ? vertex->subtreeWeight - vertex->removedWeightValue * vertex->sizeOfSubtree : 0);
}


void NNode::updateNNodeParams(NNode* vertex) {
    if(vertex) {
        vertex->sizeOfSubtree = getSize(vertex->leftChild) + getSize(vertex->rightChild) + 1;
        vertex->subtreeWeight = getSubtreeWeight(vertex->leftChild) + getSubtreeWeight(vertex->rightChild) + vertex->edgeWeight;
        vertex->subtreeMaxWeight = max(max(getMax(vertex->leftChild), getMax(vertex->rightChild)), vertex->edgeWeight);
        vertex->subtreeMinWeight = min(min(getMin(vertex->leftChild), getMin(vertex->rightChild)), vertex->edgeWeight);
    }
}

void SplayTree::_setParent(NNode* vertex, NNode* parent) {
    if(vertex) {
        vertex->parent = parent;
    }
}

void SplayTree::_keepParent(NNode* vertex) {
    _setParent(vertex->leftChild, vertex);
    _setParent(vertex->rightChild, vertex);
    NNode::updateNNodeParams(vertex);
}

void SplayTree::_rotate(NNode* parent, NNode* vertex) {
    NNode* grandParent = parent->parent;
    
    NNode::push(grandParent);
    NNode::push(parent);
    NNode::push(vertex);
    
    if(grandParent) {
        if(grandParent->leftChild == parent) {
            grandParent->leftChild = vertex;
        } else {
            grandParent->rightChild = vertex;
        }
    }
    
    if(parent->leftChild == vertex) {
        parent->leftChild = vertex->rightChild;
        vertex->rightChild = parent;
    } else {
        parent->rightChild = vertex->leftChild;
        vertex->leftChild = parent;
    }
    
    _keepParent(parent);
    _keepParent(vertex);
    
    _setParent (vertex, grandParent);
}

void SplayTree::splay(NNode* vertex){
    while(true) {
        if(!vertex->parent) {
            _root = vertex;
            _root->treePtr = this;
            return;
        }
        
        NNode* parent = vertex->parent;
        NNode* grandParent = parent->parent;
        
        if(!grandParent) {
            _rotate(parent, vertex);
            _root = vertex;
            _root->treePtr = this;
            return;
        }
        
        bool zigZigFlag = ((grandParent->leftChild == parent) == (parent->leftChild == vertex));
        
        if(zigZigFlag) {
            _rotate(grandParent, parent);
            _rotate(parent, vertex);
        } else {
            _rotate(parent, vertex);
            _rotate(grandParent, vertex);
        }
    }
}

NNode* SplayTree::find(size_t position) {
    size_t treeSize = NNode::getSize(_root);
    
    if(position >= treeSize) {
        return NULL;
        // throw std::out_of_range("out of range in SplayTree::find\n");
    }
    
    return _find(position, _root);
}

NNode* SplayTree::_find(size_t position, NNode* vertex) {
    NNode::push(vertex);
    
    size_t indexLeft = NNode::getSize(vertex->leftChild);
    
    if(position == indexLeft) {
        splay(vertex);
        return vertex;
    }
    
    if(position < indexLeft) {
        return _find(position, vertex->leftChild);
    }
    
    return _find(position - indexLeft - 1, vertex->rightChild);
}

std::pair<SplayTree*, SplayTree*> SplayTree::split(SplayTree* tree,size_t position) {
    SplayTree* leftTree = nullptr;
    SplayTree* rightTree = nullptr;
    if(tree) {
        rightTree = tree->_split(position);
        leftTree = tree;
    }
    return std::make_pair(leftTree, rightTree);
}


SplayTree* SplayTree::_split(size_t position){
    size_t treeSize = (_root ? _root->sizeOfSubtree : 0);
    
    if(position > treeSize) {
        return NULL;
        //    throw std::out_of_range("out of range in SplayTree::split\n");
    }
    
    if(position == treeSize) {
        return new SplayTree(nullptr);
    }
    
    NNode* newRoot = _find(position, _root);
    
    SplayTree* rightTree = new SplayTree(newRoot);
    
    _root = newRoot->leftChild;
    newRoot->leftChild = nullptr;
    _setParent(_root, nullptr);
    
    if(rightTree->_root) {
        rightTree->_root->treePtr = rightTree;
    }
    
    NNode::push(rightTree->_root);
    NNode::push(_root);
    
    return rightTree;
}

SplayTree* SplayTree::merge(SplayTree* leftTree, SplayTree* rightTree) {
    if(!leftTree) {
        return rightTree;
    }
    
    leftTree->_merge(rightTree);
    
    return leftTree;
}

void SplayTree::_merge(SplayTree* addedTree) {
    if(!addedTree->_root)
    {
        delete addedTree;
        addedTree = nullptr;
    }
    
    
    if(!_root) {
        _root = addedTree->_root;
        addedTree->_root = nullptr;
        delete addedTree;
        addedTree = nullptr;
        return;
    }
    
    find(_root->sizeOfSubtree - 1);
    addedTree->find(0);
    
    NNode::push(_root);
    
    _root->rightChild = addedTree->_root;
    addedTree->_root = nullptr;
    delete addedTree;
    addedTree = nullptr;
    _keepParent(_root);
}

void SplayTree::insert(int key, int position) {
    size_t treeSize = (_root ? _root->sizeOfSubtree : 0);
    
    if(position > treeSize) {
        return;
        // throw std::out_of_range("out of range in SplayTree::insert\n");
    }
    
    SplayTree* rightTree = _split(position);
    NNode* newRoot = new NNode(key);
    newRoot->leftChild = _root;
    newRoot->rightChild = rightTree->_root;
    _root = newRoot;
    _keepParent(_root);
    
    NNode::updateNNodeParams(_root);
    
    rightTree->_root = NULL;
    
    rightTree->~SplayTree();
}
