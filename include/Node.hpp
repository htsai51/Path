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
     /**
      *   @brief  Constructor of Node class.  Cost and estimated
      *           cost are initialized to integer max
      *
      *   @param  index of node in integer
      *   @param  x position of node in double
      *   @param  y position of node in double
      *   @return none
     */
     Node(int i, double x, double y)
         : index(i), parentIndex(0), xPos(x), yPos(y) {
         // initialize cost, estimateCost to MAX integer
         cost = std::numeric_limits<int>::max();
         estimateCost = std::numeric_limits<int>::max();
     }

     /**
      *   @brief  Deconstructor of Node class
      *
      *   @param  none
      *   @return none
     */
     ~Node() {}


     /**
      *   @brief  Get index of a node
      *
      *   @param  none
      *   @return index of a node in integer
     */
     int getIndex(void) { return index; }


     /**
      *   @brief  Set parent index of a node
      *
      *   @param  index of parent node in integer
      *   @return none
     */
     void setParentIndex(int index) { parentIndex = index; }


     /**
      *   @brief  Get parent index of node
      *
      *   @param  none
      *   @return parent index of node in integer
     */
     int getParentIndex(void) { return parentIndex; }


     /**
      *   @brief  Set the cost from start to this node
      *
      *   @param  cost of start to this node in double
      *   @return none
     */
     void setCost(double newCost) { cost = newCost; }


     /**
      *   @brief  Get cost of start to this node
      *
      *   @param  none
      *   @return cost of start to this node in double
     */
     double getCost(void) { return cost; }


     /**
      *   @brief  Set the estimated cost from this node to goal
      *
      *   @param  cost of node to goal in double
      *   @return none
     */
     void setEstimateCost(double newEstCost) { estimateCost = newEstCost; }


     /**
      *   @brief  Get estimated cost of node to goal
      *
      *   @param  none
      *   @return estimated cost of node to goal in double
     */
     double getEstimateCost(void) { return estimateCost; }


     /**
      *   @brief  Get x, y position of node
      *
      *   @param  none
      *   @return x, y position group by tuple in double
     */
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
