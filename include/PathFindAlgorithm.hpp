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

/** @file PathFindAlgorithm.hpp
 *  @brief Definition of class PathFindAlgorithm
 *
 *  This file contains definitions and prototypes of class
 *  pathFindAlgorithm.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#ifndef INCLUDE_PATHFINDALGORITHM_HPP_
#define INCLUDE_PATHFINDALGORITHM_HPP_

#include <vector>
#include <string>
#include "Node.hpp"
#include "Edge.hpp"
#include "Map.hpp"


#define DEFAUTL_TEST_MAP     "../data/test.csv"
#define DEFAUTL_DEFAULT_MAP  "../data/default.csv"
#define DEFAUTL_OUTPUT_MAP   "../data/out.csv"
#define DEFAUTL_OUTPUT_PATH  "../data/path.txt"

/**
 *  @brief Class that implements the basic functions
 *         reqiured for path finding algorithm.
*/
class PathFindingAlgorithm {
 public:
     /**
      *   @brief  Constructor of PathFindingAlgorithm class
      *
      *   @param  none
      *   @return none
     */
     PathFindingAlgorithm() : start(0), goal(0), totalCost(0), steps(0) {}


     /**
      *   @brief  Deconstructor of PathFindingAlgorithm class
      *
      *   @param  none
      *   @return none
     */
     ~PathFindingAlgorithm() {}

     /**
      *   @brief  Initialize map and graph node, edges
      *
      *   @param  input map file path
      *   @return true if init is successful, false otherwise
     */
     bool init(std::string);


     /**
      *   @brief  Build graph by storing map info into nodes 
      *           and edges for finding shortest path
      *  
      *   @param  none
      *   @return none
     */
     void buildGraph();


     /**
      *   @brief  Reconstruct graph by traversing from goal to start via
      *           parentIndex of nodes and store the path indices in member
      *           path which is a vector of integers
      *
      *   @param  Node pointer of goal
      *   @return none
     */
     void reconstructPath(Node*);


     /**
      *   @brief  Set start and goal indices
      *
      *   @param  start node index in int
      *   @param  goal node index in int
      *   @return true if start, goal are within map range and
      *           are not obstacle nodes, false otherwise
     */
     bool setParam(int, int);


     /**
      *   @brief  Output path in map
      *
      *   @param  option to output path in int \n
      *           0: display path in map on screen \n
      *           1: output path in map to file \n
      *           2: both display path in map on screen and save to file
      *   @return none
     */
     void outputPath(int);


     /**
      *   @brief  Output map with indices on screen
      *
      *   @param  none
      *   @return none
     */
     void outputMap();


     /**
      *   @brief  Virtual function of finding shortest path
      *
      *   @param  none
      *   @return none
     */
     virtual void computPath() {}


     /**
      *   @brief  Get shotest path indices from start to goal
      *
      *   @param  none
      *   @return shortest path indices from start to goal in vector int
     */
     std::vector<int> getPath()
                           { return path; }


     /**
      *   @brief  Get total cost of shortest path
      *
      *   @param  none
      *   @return total cost from start to goal in double
     */
     double getTotalCost()
                { return totalCost; }

     /**
      *   @brief  Get steps taken to find the shortest path
      *
      *   @param  none
      *   @return steps taken to find shortest path in int
     */
     int getSteps() { return steps; }


 protected:
     std::vector<Node> nodes;               ///< vector of nodes
     std::vector<Edge> edges;               ///< vector of edges
     int start;                             ///< start index
     int goal;                              ///< goal index
     double totalCost;                      ///< cost of shortest path
     int steps;                             ///< steps to find shortest path
     std::vector<int> path;                 ///< indices of shortest path

 private:
     Map map;                               ///< map info
};

#endif  // INCLUDE_PATHFINDALGORITHM_HPP_
