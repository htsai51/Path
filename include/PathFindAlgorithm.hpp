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

/** @file PathFindAlgorithm.hpp
 *  @brief Definition of class PathFindAlgorithm
 *
 *  This file contains definitions of class pathFindAlgorithm which
 *  implements the basic functions needed for path planning to
 *  compute the shortest path.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#ifndef INCLUDE_PATHFINDALGORITHM_HPP_
#define INCLUDE_PATHFINDALGORITHM_HPP_

#include <vector>
#include "Node.hpp"
#include "Edge.hpp"
#include "Map.hpp"


#define DEFAUTL_INPUT_MAP    "../data/test.csv"
#define DEFAUTL_OUTPUT_MAP   "../data/testout.csv"


/**
 *  @brief Class that implements the basic functions
 *         reqiured for PathFinding algorithm.
*/
class PathFindingAlgorithm {
 public:
     PathFindingAlgorithm() {}              ///< constructor
     ~PathFindingAlgorithm() {}             ///< deconstructor

     void Init();                           ///< initialize map 

     void BuildGraph();                     ///< build map by reading in map

     bool setParam(int, int);               ///< set start and goal indices

     void ReconstructPath(Node*);           ///< build shortest path

     virtual void ComputPath() {}           ///< virtual function to find
                                            ///< shortest path

     void OutputMap();                      ///< display map on screen

     void OutputPath(int);                  ///< output path on screen/file

     std::vector<int> getPath()            ///< get path vectors
                           { return path; }

     double getTotalCost()
                { return totalCost; }       ///< get path vectors

     double getSteps() { return steps; }    ///< get number of steps take to 
                                            ///< find shortest path


 protected:
     std::vector<Node> nodes;               ///< vector of nodes
     std::vector<Edge> edges;               ///< vector of edges
     int start;                             ///< start index
     int goal;                              ///< goal index
     double totalCost;                      ///< total cost of shortest path
     int steps;                             ///< total steps to find shortest path
     std::vector<int> path;                 ///< shortest path of node indices

 private:
     Map map;                               ///< map info

};

#endif  // INCLUDE_PATHFINDALGORITHM_HPP_
