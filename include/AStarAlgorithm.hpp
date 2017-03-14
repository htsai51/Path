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
#include <memory>
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
     ///< pointers to nodes in open set
     std::list<std::shared_ptr<Node>> openSet;

     ///< pointers to nodes in closed set
     std::list<std::shared_ptr<Node>> closedSet;


     /**
      *   @brief  Compute heuristic cost between start and end nodes
      *           using euclidean distance
      *
      *   @param  reference to pointer of start node
      *   @param  reference to pointer of end node
      *   @return heuristic cost estimation in double
     */
     double getHeuristicCost(std::shared_ptr<Node> &, std::shared_ptr<Node> &);


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
      *   @param  reference to the node neighbor pointer list
      *   @return none
     */
     void findNeighbors(int, std::list<std::shared_ptr<Node>> &);
};


/*
 *   @brief  Helper function, check if node is in a given list
 *  
 *   @param  node index in int
 *   @param  reference to the node pointer list to check
 *   @return true if node is in the list, false otherwise
*/
bool checkList(int, std::list<std::shared_ptr<Node>> const &);


/*
 *   @brief  Helper function, compare estimated cost of two nodes
 *  
 *   @param  pointer to first node
 *   @param  pointer to second node
 *   @return true if estimated cost of first node is lower than second node
 *           false otherwise
*/
bool compareCost(const std::shared_ptr<Node> &, const std::shared_ptr<Node> &);

#endif  // INCLUDE_ASTARALGORITHM_HPP_
