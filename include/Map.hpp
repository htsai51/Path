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

/** @file Map.hpp
 *  @brief Definition of class Map
 *
 *  This file contains definitions and prototypes of class Map
 *
 *  @author Huei Tzu Tsai
 *  @date   03/08/2017
*/

#ifndef INCLUDE_MAP_HPP_
#define INCLUDE_MAP_HPP_

#include <string>
#include <vector>


/**
 *  @brief Class definition of Map used for keeping map
 *         information for path planning.
*/
class Map {
 public:
     /**
      *   @brief  Constructor of Map class
      *
      *   @param  none
      *   @return none
     */
     Map() : startIdx(0), goalIdx(0),
             row(0), col(0), numDir(8) {}


     /**
      *   @brief  Deconstructor of Map class
      *
      *   @param  none
      *   @return none
     */
     ~Map() {}

     /**
      *   @brief  Read map info from csv file and store
      *           in mapArray
      *  
      *   @param  input file path in string
      *   @return true is reading map is successful, false otherwise
     */
     bool createMap(std::string);

     /**
      *   @brief  Output map including start, goal, and
      *           shortest path to a csv file
      *  
      *   @param  output file path in string
      *   @return true is output map is successful, false otherwise
     */
     bool saveMap(std::string, std::vector<int>&);


     /**
      *   @brief  Set start and goal indices
      *  
      *   @param  start index in int
      *   @param  goal index in int
      *   @return true if start, goal are within map range and
      *           are not obstacle nodes, false otherwise
     */
     bool setStartGoal(int, int);


     /**
      *   @brief  Display path in map on screen
      *
      *   @param  reference to vector int of path indices
      *   @return none
     */
     void displayPath(std::vector<int>&);


     /**
      *   @brief  Display map on screen
      *
      *   @param  none
      *   @return none
     */
     void displayMap();


     /**
      *   @brief  Get number of rows in map
      *
      *   @param  none
      *   @return number of rows in map in integer
     */
     int getRow(void) { return row; }


     /**
      *   @brief  Get number of columns in map
      *
      *   @param  none
      *   @return number of columns in map in integer
     */
     int getCol(void) { return col; }


     /**
      *   @brief  Get number of moving directions of a node
      *
      *   @param  none
      *   @return number of moving directions of a node in integer
     */
     int getNumDir(void) { return numDir; }


     /**
      *   @brief  Get map array
      *
      *   @param  none
      *   @return pointer to vector int of map array
     */
     std::vector<int>* getMap(void) { return &mapArray; }


     /**
      *   @brief  Get moving direction array
      *
      *   @param  none
      *   @return pointer to int of moving direction array
     */
     int* getMoveDir(void) { return moveDirection; }


 private:
     int startIdx;                                 ///< start index
     int goalIdx;                                  ///< goal index
     int row;                                      ///< numbert of rows in map
     int col;                                      ///< number of cols in map
     int numDir;                                   ///< number of direction
                                                   ///< a node can move

     std::vector<int> mapArray;                    ///< 2D map array

     int moveDirection[16] =                       ///< moving direction
                             {-1, -1,              ///< top left
                               0, -1,              ///< up
                               1, -1,              ///< top right
                              -1,  0,              ///< left
                               1,  0,              ///< right
                              -1,  1,              ///< bottom left
                               0,  1,              ///< down
                               1,  1               ///< bottom right
                             };

     /**
      *   @brief  Check if index is in shortest path
      *  
      *   @param  path index in int
      *   @param  reference to a vector int of path indices
      *   @return true if index is in shortest path
      *           false otherwise
     */
     bool isInPath(int, std::vector<int>&);
};


#endif  // INCLUDE_MAP_HPP_
