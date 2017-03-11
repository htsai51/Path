/********************************************************************
 *                                                                  *
 * Copyright (C) 2017 by Huei Tzu Tsai                              *
 *                                                                  *
 ********************************************************************/


/** @file test.cpp
 *  @brief Implementation of unit test for AStarAlgorithm
 *
 *  This file contains implementation of unit test for AStartAlgorithm
 *
 *  @author Huei Tzu Tsai
 *  @date   03/10/2017
*/

#include <iostream>
#include <vector>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "AStarAlgorithm.hpp"


using std::vector;


TEST(testSetParam1, handleOutOfBoundCondition) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::Init();

    // start, goal out of map
    ASSERT_FALSE(aStar.setParam(0, 1000));

    // start, goal is obstacle
    ASSERT_FALSE(aStar.setParam(7, 10));
}


TEST(testSetParam2, handlCorrectStartGoal) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::Init();

    // start, goal is acceptable
    ASSERT_TRUE(aStar.setParam(1, 36));

    // start, goal is acceptable
    ASSERT_TRUE(aStar.setParam(1, 15));

}


TEST(testPath, computePathShouldSucceed) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::Init();

    aStar.setParam(1, 30);

    // set weight to 1.0 for A* algorithm
    aStar.ComputPath(1.0);

    // expect path to be computed
    ASSERT_TRUE(aStar.ComputPath(1.0));
}


TEST(testCorrectness, computePathShouldBeCorrect) {
    AStarAlgorithm aStar;
    vector<int> path1;
    vector<int> path2;

    aStar.PathFindingAlgorithm::Init();

    aStar.setParam(1, 24);

    // set weight to 1.0 for A* algorithm
    aStar.ComputPath(1.0);
    path1 = aStar.PathFindingAlgorithm::getPath();

    // set weight to 0.0 for Dijkstras algorithm
    aStar.ComputPath(0.0);
    path2 = aStar.PathFindingAlgorithm::getPath();

    // make sure path matches
    EXPECT_THAT(path1, ::testing::ContainerEq(path2));
}


TEST(testCompleteness, handleUnreachableGoal) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::Init();

    // set an unreachable goal
    aStar.setParam(1, 15);
    aStar.ComputPath(1.0);

    // make sure test return fail
    ASSERT_FALSE(aStar.ComputPath(1.0));


    // set an unreachable goal
    aStar.setParam(1, 23);
    aStar.ComputPath(1.0);

    // make sure test return fail
    ASSERT_FALSE(aStar.ComputPath(1.0));
}


TEST(testRobostness, computePathShouldBeRobost) {
    AStarAlgorithm aStar;
    vector<int> path1;
    vector<int> path2;

    aStar.PathFindingAlgorithm::Init();

    // set start, goal, and compute using A*
    aStar.setParam(1, 30);
    aStar.ComputPath(1.0);

    // assert that path can be generated
    ASSERT_TRUE(aStar.ComputPath(1.0));
    path1 = aStar.PathFindingAlgorithm::getPath();

    // run compute path for same start & goal for 100 times
    // and make sure results are consistent and no crash
    for (int i = 0; i < 100; i++) {
        aStar.setParam(1, 30);
        aStar.ComputPath(1.0);

        ASSERT_TRUE(aStar.ComputPath(1.0));
        path2 = aStar.PathFindingAlgorithm::getPath();
        EXPECT_THAT(path1, ::testing::ContainerEq(path2));
    }
}

