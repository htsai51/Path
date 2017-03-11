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

/** @file Edge.hpp
 *  @brief Definition of class Edge
 *
 *  This file contains definitions of class Edge which is used to
 *  keep edge/cost information between nodes for path planning
 *  computation.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/11/2017
*/

#ifndef INCLUDE_EDGE_HPP_
#define INCLUDE_EDGE_HPP_


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

#endif  // INCLUDE_EDGE_HPP_
