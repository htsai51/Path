/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/


/** @file Map.cpp
 *  @brief Implementation of class AStarAlgorithm methods
 *
 *  This file implements class AStarAlgorithm methods and
 *  its helper functions.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <limits>
#include <string.h>

#include "Map.hpp"


using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::getline;

using std::shared_ptr;


/*
 *   @brief  Read map info from csv file and store
 *           as map data
 *  
 *   @param  input file path in string
 *   @return true is read map is successful, false otherwise
*/
bool Map::createMap(string inputFile) {
    ifstream inputFs;
    string line;
    string temp;
    int cnt = 0;

    // initialize param
    mapArray.clear();
    row = 0;
    col = 0;

    // open graph file
    inputFs.open(inputFile);

    if (inputFs.is_open()) {
        while (getline(inputFs, line)) {
            std::istringstream linestream(line);

            while (getline(linestream, temp, ',')) {
                ++cnt;
                if ((temp == "o") || (temp == "O")) {
                    mapArray.emplace_back(std::numeric_limits<int>::max());
                } else {
                    mapArray.emplace_back(std::stoi(temp));
                }
            }

            col = cnt;
            cnt = 0;

            ++row;

        }

        cout << "number of cols: " << col << endl;
        cout << "number of rows: " << row << endl;
        cout << "size of vector array: " << mapArray.size() << endl;
        inputFs.close();

        return true;
    }

    return false;
}


/*
 *   @brief  Output map including start, goal, and
 *           shortest path to csv file
 *  
 *   @param  output file path in string
 *   @return true is output map is successful, false otherwise
*/
bool Map::saveMap(string outputFile, vector<int> &path) {
    ofstream outputFs;
    int i = 0;
    int j = 0;
    int index = 0;

    outputFs.open(outputFile);

    if (outputFs.is_open()) {

        for (i = 0; i < row; ++i) {
            for (j = 0; j < col; ++j) {

                index = i * row + j + 1;

                if (index == startIdx)
                    outputFs << "S";
                else if (index == goalIdx)
                    outputFs << "G";
                else if (isInPath(index, path))
                    outputFs << "*";
                else if (mapArray[index-1] == std::numeric_limits<int>::max())
                    outputFs << "O";
                else
                    outputFs << "1";


                if (j < col-1)
                    outputFs << ",";
            }
            outputFs << endl;
        }

        outputFs.close();
        return true;
    }

    return false;
}


/*
 *   @brief  Set start and goal indices
 *  
 *   @param  start index in int
 *   @param  goal index in int
 *   @return true if start and goal are within map indices
 *           and are not obstacles, false otherwise
*/
bool Map::setStartGoal(int s, int g) {
    int minIndex = 1;
    int maxIndex = row * col;
    int index = 0;

    if ((s < minIndex) || (s > maxIndex))
        return false;

    if (g < minIndex || g > maxIndex)
        return false;

    for (auto& i : mapArray) {
        if (i == std::numeric_limits<int>::max()) {
            index = &i - &mapArray[0] - 1;
            if ((s == index) || (g == index)) {
                return false;
            }
        }
    }

    startIdx = s;
    goalIdx = g;

    return true;
}


/*
 *   @brief  Check if index is in shortest path
 *  
 *   @param  path index in int
 *   @param  vector of path indices in int
 *   @return true if index is in shortest path
 *           false otherwise
*/
bool Map::isInPath(int index, vector<int> &path) {

    for (auto n : path) {
        if (n == index)
            return true;
    }

    return false;
}


/*
 *   @brief  Display map on screen
 *  
 *   @param  vector of path indices in int
 *   @return none
*/
void Map::displayMap(vector<int> &path) {
    int x = 0;
    int y = 0;
    int index = 0;

    int displayRow = row * 2 + 1;
    int displayCol = col * 2 + 1;

    for (int i = 0; i < displayRow; ++i) {
        for (int j = 0; j < displayCol; ++j) {
            if (i % 2 == 0) {
                if (j % 2 == 0)
                    cout << " ";
                else
                    cout << "---";
            } else {
                if (j % 2 == 0)
                    cout << "|";
                else {
                    x = i/2;
                    y = j/2;

                    index = (x * col + y + 1);

                    if (index == startIdx)
                        cout << " S ";
                    else if (index == goalIdx)
                        cout << " G ";
                    else if (isInPath(index, path))
                        cout << " * ";
                    else if (mapArray[index-1] == std::numeric_limits<int>::max())
                        cout << " O ";
                    else
                        cout << " 1 ";
                }

            }
        }
        cout << endl;
    }

    return;
}


