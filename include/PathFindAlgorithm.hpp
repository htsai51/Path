/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/

/** @file PathFindAlgorithm.hpp
 *  @brief Definition of class PathFindAlgorithm
 *
 *  This file contains definitions of class pathFindAlgorithm and
 *  its component classes node and edge
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#ifndef INCLUDE_PATHFINDALGORITHM_HPP_
#define INCLUDE_PATHFINDALGORITHM_HPP_

#include <string>
#include <vector>
#include <limits>
#include <tuple>
#include "Map.hpp"





/**
 *  @brief Class that maintains map information of a node 
*/
class Node {
 public:
     Node(int i, double x, double y)                         ///< constructor
         : index(i), parentIndex(0), xPos(x), yPos(y) {
         // initialize cost, estimateCost to MAX integer
         cost = std::numeric_limits<int>::max();
         estimateCost = std::numeric_limits<int>::max();
     }
     ~Node() {}                                              ///< deconstructor

     int getIndex(void) { return index; }

     void setParentIndex(int index) { parentIndex = index; }
     double getParentIndex(void) { return parentIndex; }

     void setCost(double newCost) { cost = newCost; }
     double getCost(void) { return cost; }

     void setEstimateCost(double newEstCost) { estimateCost = newEstCost; }
     double getEstimateCost(void) { return estimateCost; }

     std::tuple<double, double> getPos(void)
         { return std::make_tuple(xPos, yPos); }

 private:
     int index;                                    ///< index
     int parentIndex;                              ///< parentIndex

     double cost;                                  ///< cost from start to node
     double estimateCost;                          ///< heuristic cost to goal

     double xPos;                                  ///< x position
     double yPos;                                  ///< y position
};


/**
 *  @brief Class that maintains map information of edge cost
 *         between start and end nodes
*/
class Edge {
 public:
     Edge(int s, int e, double c)                            ///< constructor
         : startIndex(s), endIndex(e), cost(c) {}
     ~Edge() {}

     int getStartIndex(void) { return startIndex; }
     int getEndIndex(void) { return endIndex; }
     double getCost(void) { return cost; }

 private:
     int startIndex;                                 ///< start node index
     int endIndex;                                   ///< end node index
     double cost;                                    ///< edge cost
};


/**
 *  @brief Class that implements base class of PathFinding
 *         algorithm.
*/
class PathFindingAlgorithm {
 public:
     PathFindingAlgorithm() {}              ///< constructor
     ~PathFindingAlgorithm() {}             ///< deconstructor

     void BuildGraph();                     ///< build map by reading in map


     void ReconstructPath(Node*);           ///< build shortest path


     virtual void ComputPath();             ///< virtual function to find
                                            ///< shortest path

 protected:
     std::vector<Node> nodes;               ///< vector of nodes
     std::vector<Edge> edges;               ///< vector of edges
     std::vector<int> path;                 ///< shortest path of node indices
 private:
     Map map;                               ///< map info

};

#endif  // INCLUDE_PATHFINDALGORITHM_HPP_
