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

/** @file AStarAlgorithm.cpp
 *  @brief Implementation of class AStarAlgorithm methods
 *
 *  This file implements class AStarAlgorithm methods and
 *  its helper functions.
 *
 *  AStarAlgorithm derives from PathFindAlgorithm class and implements
 *  virtual function ComputPath to compute shortest path using A star
 *  algorithm.  The heuristic estimation used in A star algorithm
 *  here is implemented using Euclidean distance.
 *  
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#include "AStarAlgorithm.hpp"
#include <math.h>
#include <iostream>
#include <limits>


using std::cout;
using std::endl;
using std::string;
using std::list;


bool AStarAlgorithm::computPath(double weight) {
    int nStep = 0;


    // cout << "*** A Star Path Searching Algorithm ***" << endl;

    // initialize
    path.clear();
    openSet.clear();
    closedSet.clear();

    // start and goal cannot be less than index lower bound
    if ((start < 1) || (goal < 1)) {
        return false;
    }

    // initialize start node's cost and heuristic cost to goal
    nodes[start-1].setCost(0);
    nodes[start-1].setEstimateCost(weight * getHeuristicCost(&nodes[start-1],
                                   &nodes[goal-1]));

    // add start node to open set
    openSet.emplace_back(&nodes[start-1]);

#if 0
    double curXPos = 0;
    double curYPos = 0;
    std::tie(curXPos, curYPos) = openSet.front()->getPos();
    cout << openSet.front()->getIndex() << " " << curXPos << " "
         << curYPos << endl;
    cout << "cost = " << openSet.front()->getCost() << " estimatedCost = "
         << openSet.front()->getEstimateCost() << endl;
#endif

    while (!openSet.empty()) {
        openSet.sort(compareCost);

        // current node in open set with lowest cost
        Node *curNode = openSet.front();

        // cout << "pop open front index: " << curNode->getIndex() << endl;

        // check if current equals to goal
        if (curNode->getIndex() == goal) {
            nStep = closedSet.size();
            totalCost = curNode->getEstimateCost();
            steps = nStep;
            // cout << "found goal in " << steps << " steps" << endl;
            // cout << "cost is " << totalCost << endl;
            reconstructPath(curNode);
            return true;
        }


        if (curNode->getCost() >= std::numeric_limits<int>::max()) {
            // if front node's cost is infinite
            // path cannot be found
            break;
        }

        // remove current from open set
        openSet.pop_front();

        // add current to closed set
        closedSet.emplace_back(curNode);

#if 0
        cout << "Closed Set:" << endl;
        for (auto& n : closedSet) {
            double xPos = 0;
            double yPos = 0;
            std::tie(xPos, yPos) = n->getPos();
            cout << n->getIndex() << "(" << xPos << "," << yPos << ")";
            cout << " (" << n->getCost() << "," << n->getEstimateCost()
                 << ")" << endl;
        }
#endif
        list<Node*> neighbors;
        findNeighbors(curNode->getIndex(), neighbors);

        // cout << "Neighbors:" << endl;

        // for each neighbor of current
        for (auto& n : neighbors) {
            // if neighbor in closed set, continue
            if (checkList(n->getIndex(), closedSet)) {
                // cout << "neighbor in closed set, continue" << endl;
                continue;
            }

            double tempCost = curNode->getCost() +
                              getCostToNeighbor(curNode->getIndex(),
                                                n->getIndex());
            // cout << "node = " << curNode->getIndex() << ", tempCost = "
            //     << tempCost << endl;


            if (!checkList(n->getIndex(), openSet)) {
                // if neighbor not in open set, add
                openSet.emplace_back(n);
            } else if (tempCost >= n->getCost()) {
                // not a better path
                continue;
            }

            // update neighbor's parent to current
            n->setParentIndex(curNode->getIndex());

            // update neighbor's cost (i.e. total cost to this node)
            // to tempCost
            n->setCost(tempCost);

            // update neighbor's goal cost (i.e. cost to goal) to
            // tempCost + heuristic estimate
            double heuristic = weight * getHeuristicCost(n, &nodes[goal-1]);
            n->setEstimateCost(tempCost + heuristic);
#if 0
            double xPos = 0;
            double yPos = 0;
            std::tie(xPos, yPos) = n->getPos();
            cout << n->getIndex() << "(" << xPos << "," << yPos << ")";
            cout << " (" << n->getCost() << "," << n->getEstimateCost() << ","
                 << heuristic << ")" << endl;
#endif
        }
    }

    // cout << "Fail to find path" << endl;
    return false;
}


double AStarAlgorithm::getHeuristicCost(Node *start, Node *end) {
    double xdiff = 0;
    double ydiff = 0;

    double startX = 0;
    double startY = 0;
    double endX = 0;
    double endY = 0;

    std::tie(startX, startY) = start->getPos();
    std::tie(endX, endY) = end->getPos();

    xdiff = startX - endX;
    ydiff = startY - endY;

    return sqrt((xdiff*xdiff) + (ydiff*ydiff));
}


double AStarAlgorithm::getCostToNeighbor(int startIndex, int endIndex) {
    double cost = 0;

    for (auto& e : edges) {
        if (e.getStartIndex() == startIndex && e.getEndIndex() == endIndex) {
            cost = e.getCost();
            break;
        }
    }

    return cost;
}


void AStarAlgorithm::findNeighbors(int index, list<Node*> &neighbors) {
    for (auto& e : edges) {
        if (e.getStartIndex() == index) {
            neighbors.emplace_back(&nodes[e.getEndIndex()-1]);
        }
    }

    return;
}


bool checkList(int index, list<Node*> const &nodes) {
    bool found = false;

    for (auto& n : nodes) {
        if (n->getIndex() == index) {
            found = true;
            break;
        }
    }

    return found;
}


bool compareCost(Node *first, Node *second) {
    if (first->getEstimateCost() < second->getEstimateCost())
        return true;
    else
        return false;
}


