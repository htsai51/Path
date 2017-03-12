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

/**
 *  @file main.cpp
 *  @brief Initial file of path finding demo program
 *
 *  This file contains path finding demo program's
 *  main() function.
 *
 *  This program demonstrate the function of path finding using
 *  Astar algorithm.
 *
 *
 *  @author Huei Tzu Tsai
 *  @date   03/11/2017
*/

#include <iostream>
#include <string>
#include <vector>
#include "AStarAlgorithm.hpp"

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;


/*
 *   @brief  path demo program entrypoint
 *  
 *   @param  void
 *   @return integer 0 upon exit success \n
             integer -1 upon exit failure
*/
int main(void) {
    string mapFile;
    int start = 1;
    int goal = 2;
    int option = 0;
    double weight = 0.0;
    vector<int> path;
    AStarAlgorithm aStar;

    cout << "Please enter map path (or ctl+d to use default):" << endl;

    cin >> mapFile;
    cin.clear();

    if (mapFile.empty())
        mapFile = DEFAUTL_DEFAULT_MAP;

    if (!aStar.PathFindingAlgorithm::init(mapFile)) {
        cout << "Fail to read map.  Please try again or use default map."
             << endl;
        return -1;
    }

    aStar.PathFindingAlgorithm::outputMap();

    cout << "Please enter start, goal indices" << endl;

    cin >> start >> goal;
    cin.clear();

    cout << "start is " << start << endl;
    cout << "goal is " << goal << endl;

    if (aStar.PathFindingAlgorithm::setParam(start, goal) == false) {
        cout << "Start or goal is out of map or obstacle, please try again."
             << endl;
        return -1;
    }

    weight = 0.0;

    if (!aStar.computPath(weight)) {
        cout << "Fail to find path" << endl;
        return -1;
    }

    path = aStar.PathFindingAlgorithm::getPath();
    cout << "Dijkstra's Shortest Path:";

    // save into vector from start to goal
    for (auto& n : path) {
        cout <<  " " << n;
    }
    cout << endl;

    cout << "Dijkstra's total cost is "
         << aStar.PathFindingAlgorithm::getTotalCost() << endl;

    cout << "Dijkstra's total step is "
         << aStar.PathFindingAlgorithm::getSteps() << endl;

    // Init again to clear variables
    aStar.PathFindingAlgorithm::init(mapFile);
    path.clear();

    // Compute using A star
    weight = 1.0;
    if (!aStar.computPath(weight)) {
        cout << "Fail to find path" << endl;
        return -1;
    }

    path = aStar.PathFindingAlgorithm::getPath();
    cout << "A Star Shortest Path:";

    // save into vector from start to goal
    for (auto& n : path) {
        cout <<  " " << n;
    }
    cout << endl;

    cout << "A Star total cost is "
         << aStar.PathFindingAlgorithm::getTotalCost() << endl;

    cout << "A Star total step is " << aStar.PathFindingAlgorithm::getSteps()
         << endl;

    cout << "Output A Star Path Option:" << endl;
    cout << "0: path in map on screen" << endl;
    cout << "1: path output to " << DEFAUTL_OUTPUT_PATH << endl;
    cout << "   map output to " << DEFAUTL_OUTPUT_MAP << endl;
    cout << "2: both" << endl;

    cin >> option;
    cin.clear();

    aStar.PathFindingAlgorithm::outputPath(option);

    return 0;
}
