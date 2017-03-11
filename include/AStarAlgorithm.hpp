/********************************************************************
MIT License

Copyright (c) 2017 Huei-Tzu Tsai

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
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

     bool setParam(int, int);                    ///< set parameters
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
