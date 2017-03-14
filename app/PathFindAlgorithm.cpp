/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017 Huei-Tzu Tsai
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/


/** @file PathFindAlgorithm.cpp
 *  @brief Implementation of class PathFindAlgorithm methods
 *
 *  This file implements methods in base class of PathFindAlgorithm.
 *
 *  PathFindAlgorithm is the base class for path finding algorithm.
 *  It supports building map/graph into nodes, edges for computing
 *  shortest path, reconstructing path from goal to start, and
 *  displaying map with or without shortest path on screen.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#include <iostream>
#include <fstream>
#include <list>
#include <limits>
#include "PathFindAlgorithm.hpp"
#include "Map.hpp"


using std::cout;
using std::endl;
using std::string;
using std::list;
using std::vector;
using std::ofstream;


bool PathFindingAlgorithm::init(string input) {
    if (map.createMap(input)) {
        nodes.clear();
        edges.clear();
        path.clear();

        totalCost = 0;

        buildGraph();
        return true;
    }

    return false;
}


void PathFindingAlgorithm::buildGraph(void) {
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
    for (i = 0, index = 1; i < n; ++i) {  // row, y
        for (j = 0; j < m; ++j) {  // column, x
            // create a node and add to node vector

            // cout << index << " (" << i << ", " << j << ")" << endl;

            nodes.emplace_back(index, i, j);
            ++index;
        }
    }

    // cout << endl << "Generating edges" << endl;

    // generate edges
    for (i = 0; i < n; ++i) {  // row, y
        for (j = 0; j < m; ++j) {  // column, x
            int numDir = map.getNumDir();
            int startIdx = 0;
            int endIdx = 0;
            double cost = 0;
            double diagCost = 0;

            for (k = 0; k < numDir; ++k) {
                int neighborX = j + *(dir+k*2);
                int neighborY = i + *(dir+k*2+1);

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
                    if ((k == 0) || (k == 2) || (k == 5) || (k == 7))
                       diagCost = 1.5;
                    else
                       diagCost = 1.0;

                    if (cost < std::numeric_limits<int>::max())
                        cost = cost * diagCost;

                    // cout << k << " (" << startIdx << "," << endIdx << ") "
                    //      << cost << endl;

                    edges.emplace_back(startIdx, endIdx, cost);
                }
            }
        }
    }

    return;
}


void PathFindingAlgorithm::reconstructPath(Node *node) {
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
    for (auto& n : tempPath) {
        // cout << n->getIndex() << " ";
        path.emplace_back(n->getIndex());
    }

    // cout << endl;


    return;
}


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


void PathFindingAlgorithm::outputPath(int option) {
    ofstream outputFs;

    switch (option) {
         // display map on screen
        case 0:
            map.displayPath(path);
            break;

        // output to file
        case 1:
            map.saveMap(DEFAUTL_OUTPUT_MAP, path);

            outputFs.open(DEFAUTL_OUTPUT_PATH);

            if (outputFs.is_open()) {
                for (auto& n : path) {
                    outputFs << n << endl;
                }
                outputFs.close();
            }
            break;

        case 2:
        default:
            map.displayPath(path);
            map.saveMap(DEFAUTL_OUTPUT_MAP, path);
            outputFs.open(DEFAUTL_OUTPUT_PATH);

            if (outputFs.is_open()) {
                for (auto& n : path) {
                    outputFs << n << endl;
                }
                outputFs.close();
            }
            break;
    }
}


void PathFindingAlgorithm::outputMap() {
    map.displayMap();
    return;
}
