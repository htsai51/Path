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


#include <iostream>
#include <string>
#include <vector>
#include "AStarAlgorithm.hpp"
#include "Map.hpp"

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

int main(void) {
    int start = 1;
    int goal = 2;
    double weight = 1.0;
    vector<int> path;
    AStarAlgorithm aStar;
    

    aStar.PathFindingAlgorithm::Init();

    aStar.PathFindingAlgorithm::OutputMap();

    cout << "Please enter start, goal indices" << endl;

    cin >> start >> goal;

    cout << "start is " << start << endl;
    cout << "goal is " << goal << endl;

    if (aStar.setParam(start, goal) == false) {
        cout << "Start or goal is out of map or obstacle, please try again." << endl;
    } else {
        if (aStar.ComputPath(weight)) {
            path = aStar.PathFindingAlgorithm::getPath();

            cout << "Shortest Path: ";

            // save into vector from start to goal
            for (auto& n : path) {
                cout <<  " " << n;
            }

            cout << endl;

            cout << "Total cost is " << aStar.PathFindingAlgorithm::getTotalCost()
                 << endl;

            cout << "Total step is " << aStar.PathFindingAlgorithm::getSteps()
                 << endl;

            aStar.PathFindingAlgorithm::OutputPath(2);
        } else {
            cout << "Fail to find path" << endl;
        }
    }

    return 0;
}
