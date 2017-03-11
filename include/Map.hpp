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

/** @file Map.hpp
 *  @brief Definition of class Map
 *
 *  This file contains definitions of class Map
 *
 *  @author Huei Tzu Tsai
 *  @date   03/08/2017
*/

#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <string>
#include <vector>


/**
 *  @brief Class definition of Map used for keeping 2D array
 *         map information for path planning.
*/
class Map {
 public:
     Map() : startIdx(0), goalIdx(0),
             row(0), col(0), numDir(8) {}               ///< constructor
     ~Map() {}                                          ///< deconstructor

     bool createMap(std::string);                       ///< create map from file
     bool saveMap(std::string, std::vector<int>&);      ///< save map to file
     bool setStartGoal(int, int);                       ///< set start & goal
     int getRow(void) { return row; }                   ///< get rows of map
     int getCol(void) { return col; }                   ///< get cols of map
     int getNumDir(void) { return numDir; }             ///< get num of dir

     std::vector<int>* getMap(void)
                           { return &mapArray; }        ///< get map
     int* getMoveDir(void) { return moveDirection; }    ///< get move direction
     void displayPath(std::vector<int>&);               ///< display path in map
     void displayMap();                                 ///< display map


 private:
     int startIdx;
     int goalIdx;
     int row;
     int col;
     int numDir;
     std::vector<int> mapArray;
     int moveDirection[16] = {-1, -1, ///< top left
                               0, -1, ///< up
                               1, -1, ///< top right
                              -1,  0, ///< left
                               1,  0, ///< right
                              -1,  1, ///< bottom left
                               0,  1, ///< down
                               1,  1  ///< bottom right
                              };

     bool isInPath(int, std::vector<int>&);             ///< check if index is
                                                        ///< in path

};


#endif  // INCLUDE_MAP_HPP_
