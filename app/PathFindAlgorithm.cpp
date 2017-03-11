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
#include <list>
#include <limits>
#include "PathFindAlgorithm.hpp"
#include "Map.hpp"


using std::cout;
using std::endl;
using std::string;
using std::list;
using std::vector;


/**
 *   @brief  Initialize map and graph
 *  
 *   @param  
 *   @param  
 *   @return none
*/
void PathFindingAlgorithm::Init(void) {
    nodes.clear();
    edges.clear();
    path.clear();

    map.createMap(DEFAUTL_INPUT_MAP);
    BuildGraph();
}


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
    
    // cout << endl << "PathFindingAlgorithm::BuildGraph" << endl;
    
    mapArray = map.getMap();
    if (mapArray == nullptr)
        return;

    dir = map.getMoveDir();
    if (dir == nullptr)
        return;

    // cout << "Generating nodes" << endl;


    int n = map.getRow();
    int m = map.getCol();


    // cout << "map row " << n << endl;
    // cout << "map column " << m << endl;

    // generate nodes
    for (i = 0, index = 1; i < n; ++i) { // row, y
        for (j = 0; j < m; ++j) { // column, x
            // create a node and add to node vector

            // cout << index << " (" << i << ", " << j << ")" << endl;
        
            nodes.emplace_back(index, i, j);
            ++index;
        }
    }

    // cout << endl << "Generating edges" << endl;

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

                    // cout << k << " (" << startIdx << "," << endIdx << ") " << cost << endl;

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
void PathFindingAlgorithm::ReconstructPath(Node *node) {
    list<Node*> tempPath;
    Node *temp = node;

    if (temp == nullptr)
        return;

    // reconstruct path using node's parent pointer
    while (temp != nullptr) {
        // add the start node and break
        if (temp->getParentIndex() == 0) {
            tempPath.emplace_front(temp);
            break;
        }

        tempPath.emplace_front(temp);
        temp = &nodes[temp->getParentIndex()-1];
    }


    // cout << "Shortest Path: ";

    // save into vector from start to goal
    for (auto n : tempPath) {
        // cout << n->getIndex() << " ";
        path.emplace_back(n->getIndex());
    }

    // cout << endl;


    return;
}


/*
 *   @brief  Set start and goal indices
 *  
 *   @param  start node index in int
 *   @param  goal node index in int
 *   @return true if start, goal are within map indices and
 *           are not obstacle nodes, false otherwise
*/
bool PathFindingAlgorithm::setParam(int s, int g) {
    // set start and goal indices
    if (map.setStartGoal(s, g)) {
        start = s;
        goal = g;
        return true;
    } else {
        return false;
    }
}


/*
 *   @brief  Output result on screen or file or both
 *  
 *   @param  start node index in int
 *   @param  goal node index in int
 *   @return none
*/
void PathFindingAlgorithm::OutputPath(int option) {

    switch(option) {
        case 0: ///< display map on screen
            map.displayPath(path);
            break;

        case 1: ///< output to file
            map.saveMap(DEFAUTL_OUTPUT_MAP, path);
            break;

        case 2:
        default:
            map.displayPath(path);
            map.saveMap(DEFAUTL_OUTPUT_MAP, path);
            break;
    }
}


/*
 *   @brief  Output map on screen
 *  
 *   @param  none
 *   @return none
*/
void PathFindingAlgorithm::OutputMap() {

    map.displayMap();
    return;
}
