/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/


/** @file PathFindAlgorithm.cpp
 *  @brief Implementation of class PathFindAlgorithm methods
 *
 *  This file implements methods in base class of PathFindAlgorithm
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#include <iostream>
#include <limits>
#include "PathFindAlgorithm.hpp"


using std::cout;
using std::endl;
using std::string;
using std::vector;


/**
 *   @brief  Build graph with nodes and edges for PathFindingAlgorithm
 *  
 *   @param  
 *   @param  
 *   @return none
*/
void PathFindingAlgorithm::BuildGraph(void) {

    vector<int>* mapArray = nullptr;
    int* dir = nullptr;
    int i = 0;
    int j = 0;
    int k = 0;
    int index = 1;
    
    cout << endl << "PathFindingAlgorithm::BuildGraph" << endl;
    
    mapArray = map.getMap();
    if (mapArray == nullptr)
        return;

    dir = map.getMoveDir();
    if (dir == nullptr)
        return;

    cout << "Generating nodes" << endl;


    int n = map.getRow();
    int m = map.getCol();


    cout << "map row " << n << endl;
    cout << "map column " << m << endl;

    // generate nodes
    for (i = 0, index = 1; i < n; ++i) { // row, y
        for (j = 0; j < m; ++j) { // column, x
            // create a node and add to node vector

            cout << index << " (" << i << ", " << j << ")" << endl;
        
            nodes.emplace_back(index, i, j);
            ++index;
        }
    }

    cout << endl << "Generating edges" << endl;

    // generate edges
    for (i = 0; i < n; ++i) { // row, y
        for (j = 0; j < m; ++j) { // column, x

            int numDir = map.getNumDir();
            int neighborX = 0;
            int neighborY = 0;
            int startIdx = 0;
            int endIdx = 0;
            double cost = 0;
            double diagCost = 0;
    
            for (k = 0; k < numDir; ++k) {
                neighborX = j + *(dir+k*2);
                neighborY = i + *(dir+k*2+1);

                // cout << "(" << i << "," << j << ") (" << neighborX
                // << "," << neighborY << ")" << endl;

                if ((neighborX >= 0) && (neighborY >= 0) &&
                	  (neighborX < m) && (neighborY < n)) {
                    // add node to edge list
                    startIdx = i * m + j + 1;
                    endIdx = neighborY * m + neighborX + 1;

                    // set cost for moving between nodes
                    cost = (*mapArray)[endIdx-1];

                    // setting cost to 1.5x for diagonal movement
                    if ((k==0) || (k==2) || (k==5) || (k==7))
                       diagCost = 1.5;
                    else
                       diagCost = 1.0;

                    if (cost < std::numeric_limits<int>::max())
                        cost = cost * diagCost;

                    cout << k << " (" << startIdx << "," << endIdx << ") " << cost << endl;

                    edges.emplace_back(startIdx, endIdx, cost);
                }
            }
        }
    }

    return;
}


/*
 *   @brief  Reconstruct graph by traversing from goal to start via 
 *           parentIndex of nodes
 *  
 *   @param  
 *   @param  
 *   @return none
*/
void PathFindingAlgorithm::ReconstructPath(string pathFile, Node *node) {
    return;
}
