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

/** @file Node.hpp
 *  @brief Definition of class Node
 *
 *  This file contains definitions of class Node which is used to
 *  keep node information for path planning computation
 *
 *  @author Huei Tzu Tsai
 *  @date   03/11/2017
*/

#ifndef INCLUDE_NODE_HPP_
#define INCLUDE_NODE_HPP_

#include <string>
#include <vector>
#include <limits>
#include <tuple>


/**
 *  @brief Class that maintains node information in a map
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

#endif  // INCLUDE_NODE_HPP_
