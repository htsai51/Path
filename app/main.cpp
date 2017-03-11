/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
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
