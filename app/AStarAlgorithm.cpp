/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/


/** @file AStarAlgorithm.cpp
 *  @brief Implementation of class AStarAlgorithm methods
 *
 *  This file implements class AStarAlgorithm methods and
 *  its helper functions.
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



/*
 *   @brief  Set parameters for AStar algorithm
 *  
 *   @param  start node index in int
 *   @param  end node index in int
 *   @return true if start, goal node is within map boundaries
 *           and it is not obstacle node, false otherwise
*/
bool AStarAlgorithm::setParam(int s, int g) {
    // call parent function to update start, goal
    // as well as map info
    return PathFindingAlgorithm::setParam(s, g);
}


/*
 *   @brief  Compute heuristic cost between start and end nodes
 *           using euclidean distance
 *  
 *   @param  reference pointer to start node
 *   @param  reference pointer to end node
 *   @return heuristic cost estimation
*/
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


/*
 *   @brief  Get the cost between two given indices
 *  
 *   @param  start node index in int
 *   @param  end node index in int
 *   @return edge cost between two indices in double
*/
double AStarAlgorithm::getCostToNeighbor(int startIndex, int endIndex) {
    double cost = 0;

    for (auto e : edges) {
        if (e.getStartIndex() == startIndex && e.getEndIndex() == endIndex) {
            cost = e.getCost();
            break;
        }
    }

    return cost;
}


/*
 *   @brief  Find the list of neighbors for given node index
 *  
 *   @param  node index in int
 *   @param  reference to the node neighbor list to return
 *   @return none
*/
void AStarAlgorithm::findNeighbors(int index, list<Node*> &neighbors) {
    for (auto e : edges) {
        if (e.getStartIndex() == index) {
            neighbors.emplace_back(&nodes[e.getEndIndex()-1]);
        }
    }

    return;
}


/*
 *   @brief  Helper function, check if node is in a given list
 *  
 *   @param  node index in int
 *   @param  reference to the node list to check
 *   @return true if node is in the list, false otherwise
*/
bool checkList(int index, list<Node*> const &nodes) {
    bool found = false;

    for (auto n : nodes) {
        if (n->getIndex() == index) {
            found = true;
            break;
        }
    }

    return found;
}


/*
 *   @brief  Helper function, compare estimated cost of two nodes
 *  
 *   @param  index of first node in int
 *   @param  index of second node in int
 *   @return true if estimated cost of first node is lower than second node
 *           false otherwise
*/
bool compareCost(Node *first, Node *second) {
    if (first->getEstimateCost() < second->getEstimateCost())
        return true;
    else
        return false;
}


/**
 *   @brief  Compute shortest path given start, goal nodes indices,
 *           and weight for heuristic estimates
 *  
 *   @param  none
 *   @return true if shortest path can be found
 *           false otherwise
*/
bool AStarAlgorithm::ComputPath(double weight) {
    int nStep = 0;


    // cout << "*** A Star Path Searching Algorithm ***" << endl;

    // clear path vector
    path.clear();

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
            ReconstructPath(curNode);
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
        for (auto n : closedSet) {
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
        for (auto n : neighbors) {
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


