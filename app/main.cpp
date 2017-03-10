/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/


#include <iostream>
#include <string>
#include "AStarAlgorithm.hpp"
#include "Map.hpp"

using std::cout;
using std::cin;
using std::endl;
using std::string;

int main(void) {
    AStarAlgorithm aStar;

    int start = 1;
    int goal = 30;
    double weight = 1.0;

    aStar.PathFindingAlgorithm::Init();

    aStar.setParam(start, goal);

    aStar.ComputPath(weight);

    aStar.PathFindingAlgorithm::Output(2);

    return 0;
}
