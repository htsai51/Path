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


/** @file Map.cpp
 *  @brief Implementation of class Map and its methods to maintain
 *         map info of a path finding algorithm
 *
 *  This file implements methods in class Map.
 *
 *  Map class supports creating map from csv file, saving map with
 *  path from start to goal into csv file, and displaying map on
 *  the screen.
 *
 *  @author Huei Tzu Tsai
 *  @date   03/07/2017
*/


#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <memory>
#include <limits>

#include "Map.hpp"

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::ifstream;
using std::ofstream;
using std::getline;
using std::numeric_limits;

bool Map::createMap(string inputFile) {
    ifstream inputFs;
    string line;
    string temp;

    // initialize param
    mapArray.clear();
    row = 0;
    col = 0;

    // open graph file
    inputFs.open(inputFile);

    if (inputFs.is_open()) {
        while (getline(inputFs, line)) {
            int cnt = 0;
            std::istringstream linestream(line);

            while (getline(linestream, temp, ',')) {
                ++cnt;
                if ((temp == "o") || (temp == "O")) {
                    mapArray.emplace_back(numeric_limits<int>::max());
                } else {
                    mapArray.emplace_back(std::stoi(temp));
                }
            }

            col = cnt;
            cnt = 0;

            ++row;
        }

        // cout << "number of cols: " << col << endl;
        // cout << "number of rows: " << row << endl;
        // cout << "size of vector array: " << mapArray.size() << endl;
        inputFs.close();

        return true;
    }

    return false;
}


bool Map::saveMap(string outputFile, vector<int> &path) {
    ofstream outputFs;

    outputFs.open(outputFile);

    if (outputFs.is_open()) {
        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < col; ++j) {
                int index = i * row + j + 1;

                if (index == startIdx)
                    outputFs << "S";
                else if (index == goalIdx)
                    outputFs << "G";
                else if (isInPath(index, path))
                    outputFs << "*";
                else if (mapArray[index-1] == numeric_limits<int>::max())
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


bool Map::setStartGoal(int s, int g) {
    int minIndex = 1;
    int maxIndex = row * col;
    int index = 0;

    if ((s < minIndex) || (s > maxIndex))
        return false;

    if (g < minIndex || g > maxIndex)
        return false;

    for (auto& i : mapArray) {
        if (i == numeric_limits<int>::max()) {
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


void Map::displayPath(vector<int> &path) {
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
                if (j % 2 == 0) {
                    cout << "|";
                } else {
                    x = i/2;
                    y = j/2;

                    index = (x * col + y + 1);

                    if ((index == startIdx) && (index == goalIdx))
                        cout << "S/G";
                    else if (index == startIdx)
                        cout << " S ";
                    else if (index == goalIdx)
                        cout << " G ";
                    else if (isInPath(index, path))
                        cout << " * ";
                    else if (mapArray[index-1] == numeric_limits<int>::max())
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


void Map::displayMap(void) {
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
                if (j % 2 == 0) {
                    cout << "|";
                } else {
                    x = i/2;
                    y = j/2;

                    index = (x * col + y + 1);

                    if (mapArray[index-1] == numeric_limits<int>::max())
                        cout << " O ";
                    else
                        cout << std::setw(3) << index;
                }
            }
        }
        cout << endl;
    }

    return;
}


bool Map::isInPath(int index, vector<int> &path) {
    for (auto& n : path) {
        if (n == index)
            return true;
    }

    return false;
}
