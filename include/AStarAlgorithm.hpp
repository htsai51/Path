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

/** @file AStarAlgorithm.hpp
 *  @brief Definition of class AStarAlgorithm
 *
 *  This file contains definitions and prototypes of class AStarAlgorithm
 *  and its helper functions.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#ifndef INCLUDE_ASTARALGORITHM_HPP_
#define INCLUDE_ASTARALGORITHM_HPP_


#include <list>
#include "PathFindAlgorithm.hpp"


/**
 *  @brief Class definition of AStarAlgorithm class which is derived
 *         from base class PathFindingAlgorithm for path planning.
*/
class AStarAlgorithm : public PathFindingAlgorithm {
 public:
     /**
      *   @brief  Constructor of AStarAlgorithm class
      *
      *   @param  none
      *   @return none
     */
     AStarAlgorithm() {}


     /**
      *   @brief  Deconstructor of AStarAlgorithm class
      *
      *   @param  none
      *   @return none
     */
     ~AStarAlgorithm() {}


     /**
      *   @brief  Compute shortest path using given start, goal nodes
      *           indices, and weight for heuristic estimates
      *
      *   @param  weight of heuristic function in double
      *   @return true if shortest path can be found, false otherwise
     */
     bool computPath(double);

 private:
     std::list<Node*> openSet;             ///< pointers to nodes in open set
     std::list<Node*> closedSet;           ///< pointers to nodes in closed set


     /**
      *   @brief  Compute heuristic cost between start and end nodes
      *           using euclidean distance
      *
      *   @param  reference pointer to start node
      *   @param  reference pointer to end node
      *   @return heuristic cost estimation in double
     */
     double getHeuristicCost(Node *, Node *);


     /**
      *   @brief  Get the cost between two given indices
      *
      *   @param  start node index in int
      *   @param  end node index in int
      *   @return edge cost between two indices in double
     */
     double getCostToNeighbor(int, int);


     /**
      *   @brief  Find the list of neighbors for given node index
      *
      *   @param  node index in int
      *   @param  reference to the node neighbor list to return
      *   @return none
     */
     void findNeighbors(int, std::list<Node*> &);
};


/*
 *   @brief  Helper function, check if node is in a given list
 *  
 *   @param  node index in int
 *   @param  reference to the node list to check
 *   @return true if node is in the list, false otherwise
*/
bool checkList(int, std::list<Node*> const &);


/*
 *   @brief  Helper function, compare estimated cost of two nodes
 *  
 *   @param  node pointer to first node
 *   @param  node pointer to second node
 *   @return true if estimated cost of first node is lower than second node
 *           false otherwise
*/
bool compareCost(Node *, Node *);

#endif  // INCLUDE_ASTARALGORITHM_HPP_
