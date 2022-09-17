//
//  splaytree.h
//  Dinic's Algorithm with Link-Cut-Tree
//
//  Created by Сергей Миллер on 10.10.15.
//  Copyright © 2015 Сергей Миллер. All rights reserved.
//

#ifndef splaytree_h
#define splaytree_h

#include <cstdio>
#include <vector>

using std::min;
using std::max;

class NNode;
class SplayTree;

const double INF = 1e16;

class NNode{
    friend class SplayTree;
    friend class LinkCutTree;
public:
    static void removeWeight(double value, NNode* vertex);
    static void updateNNodeParams(NNode* vertex);
    static void recursiveDelete(NNode* vertex);
    static void push(NNode* vertex);
    static void reverse(NNode* vertex);
    
    static size_t getSize(NNode* vertex);
    static double getMin(NNode* vertex);
    static double getMax(NNode* vertex);
    static double getSum(NNode* vertex);
    static size_t getKey(NNode* vertex);
    static double getWeight(NNode* vertex);
    static double getSubtreeWeight(NNode* vertex);
    
    NNode(size_t key, size_t weight = 0);
private:
    size_t key;
    double edgeWeight;
    double subtreeWeight;
    size_t sizeOfSubtree;
    double subtreeMaxWeight;
    double subtreeMinWeight;
    double removedWeightValue;
    
    bool reverseFlag;
    
    NNode* leftChild;
    NNode* rightChild;
    NNode* parent;
    NNode* link;
    
    SplayTree* treePtr;
};

class SplayTree
{
    friend class LinkCutTree;
private:
    NNode* _find(size_t position, NNode* vertex);
    void insert(int key, int position);
    void remove(int position);
    void _keepParent(NNode* vertex);
    void _setParent(NNode* parent, NNode* vertex);
    void _rotate(NNode* parent, NNode* vertex);
    void _merge(SplayTree* addedTree); //added tree is right merged tree
    SplayTree* _split(size_t position); //returned tree is tight splited tree
    
    NNode* _root;
public:
    SplayTree(NNode* root);
    ~SplayTree();
    
    NNode* find(size_t position);
    
    static SplayTree* merge(SplayTree* leftTree, SplayTree* rightTree);
    static std::pair<SplayTree*, SplayTree*> split(SplayTree* tree,size_t position);
    
    void splay(NNode* vertex);
    
    NNode* getRoot() { return _root; };
};


#endif /* splaytree_h */