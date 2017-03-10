/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/

/** @file AStarAlgorithm.hpp
 *  @brief Definition of class AStarAlgorithm
 *
 *  This file contains definitions of class AStarAlgorithm
 *  and its helper functions
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#ifndef INCLUDE_ASTARALGORITHM_HPP_
#define INCLUDE_ASTARALGORITHM_HPP_


#include <list>
#include "PathFindAlgorithm.hpp"


/**
 *  @brief Class definition of A star algorithm which is derived
 *         from base class PathFinding algorithm for path searching.
*/
class AStarAlgorithm : public PathFindingAlgorithm {
 public:
     AStarAlgorithm() {}                         ///< constructor
     ~AStarAlgorithm() {}                        ///< deconstructor

     void setParam(int, int);                    ///< set parameters
     bool ComputPath(double);                    ///< compute shortest path

 private:
     std::list<Node*> openSet;             ///< pointers to nodes in open set
     std::list<Node*> closedSet;           ///< pointers to nodes in closed set

     double getHeuristicCost(Node *, Node *);       ///< compute heuristic cost
     double getCostToNeighbor(int, int);            ///< get cost to neighbor

     void findNeighbors(int, std::list<Node*> &);   ///< find neighbors of node
};


// helper functions
bool checkList(int, std::list<Node*> const &);   ///< check if node in list
bool compareCost(Node *, Node *);                ///< compare cost in list


#endif  // INCLUDE_ASTARALGORITHM_HPP_
