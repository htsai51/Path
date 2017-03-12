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

/** @file test.cpp
 *  @brief Implementation of unit test for AStarAlgorithm
 *
 *  This file contains implementation of unit test for AStartAlgorithm
 *
 *  @author Huei Tzu Tsai
 *  @date   03/10/2017
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

#include "AStarAlgorithm.hpp"

using std::vector;


/**
 *   @brief  Check createMap function error handling by passing a
 *           non-existing input map path \n
 *           Test expects FALSE as map should NOT be created successfully
 *
 *   @param  none
 *   @return none
*/
TEST(testCreateMap1, handleCreateMapError) {
    AStarAlgorithm aStar;

    ASSERT_FALSE(aStar.PathFindingAlgorithm::init("unknown.csv"));
}


/**
 *   @brief  Check createMap function by passing test map \n
 *           Test expects TRUE as map should be created successfully
 *
 *   @param  none
 *   @return none
*/
TEST(testCreateMap2, handleCreateMap) {
    AStarAlgorithm aStar;

    ASSERT_TRUE(aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP));
}


/**
 *   @brief  Check setParam function by setting feasible start,
 *           goal indices \n
 *           Test expects TRUE as start and goal are within map
 *           and are not obstacle nodes
 *
 *   @param  none
 *   @return none
*/
TEST(testSetParam1, handleSetParam) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    // start, goal is acceptable
    ASSERT_TRUE(aStar.PathFindingAlgorithm::setParam(1, 36));

    // start, goal is acceptable
    ASSERT_TRUE(aStar.PathFindingAlgorithm::setParam(1, 1));
}


/**
 *   @brief  Check error handling of setParam by setting start,
 *           goal indices to out of bound or obstacle nodes \n
 *           Test expects FALSE return
 *
 *   @param  none
 *   @return none
*/
TEST(testSetParam2, handleError) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    // start, goal out of map
    ASSERT_FALSE(aStar.PathFindingAlgorithm::setParam(0, 1000));

    // start, goal is obstacle
    ASSERT_FALSE(aStar.PathFindingAlgorithm::setParam(7, 10));
}


/**
 *   @brief  Check computePath function a given start,
 *           goal nodes that has a feasible path \n
 *           Test expects TRUE as there should be a feasible path
 *
 *   @param  none
 *   @return none
*/
TEST(testPath1, computePathShouldPass) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    aStar.PathFindingAlgorithm::setParam(1, 30);

    // expect path to be computed
    ASSERT_TRUE(aStar.computPath(1.0));
}


/**
 *   @brief  Check computePath function can find path for
 *           setting start to goal node \n
 *           Test expects TRUE
 *
 *   @param  none
 *   @return none
*/
TEST(testPath2, computePathShouldPass) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    aStar.PathFindingAlgorithm::setParam(1, 1);

    // expect path to be computed
    ASSERT_TRUE(aStar.computPath(1.0));
}


/**
 *   @brief  Check computePath function computes shortest
 *           path by comparing cost result to Dijkstra's
 *           result \n
 *           Test expects Astar and Dijkstra's computes the
 *           same cost for shortest path
 *
 *   @param  none
 *   @return none
*/
TEST(testCorrectness, computeCostShouldMatch) {
    AStarAlgorithm aStar;
    double cost1 = 0;
    double cost2 = 0;

    // init to setup map and clear variables
    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);
    aStar.PathFindingAlgorithm::setParam(1, 24);

    // set weight to 1.0 for A* algorithm
    aStar.computPath(1.0);
    cost1 = aStar.PathFindingAlgorithm::getTotalCost();

    // init again to clear variables
    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);
    aStar.PathFindingAlgorithm::setParam(1, 24);

    // set weight to 0.0 for Dijkstra's algorithm
    aStar.computPath(0.0);
    cost2 = aStar.PathFindingAlgorithm::getTotalCost();

    // make sure path cost matches
    ASSERT_EQ(cost1, cost2);
}


/**
 *   @brief  Check computePath returns fail for a given
 *           start and goal nodes which do not have a
 *           feasible path \n
 *           Test expects FALSE as there is no feasible path
 *
 *   @param  none
 *   @return none
*/
TEST(testCompleteness, handleUnreachableGoal) {
    AStarAlgorithm aStar;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    // set an unreachable goal
    aStar.PathFindingAlgorithm::setParam(1, 15);
    aStar.computPath(1.0);

    // make sure test return fail
    ASSERT_FALSE(aStar.computPath(1.0));
}


/**
 *   @brief  Check computePath consistently finds the same
 *           path for a given start, goal nodes \n
 *           Test expects path computed are the same
 *
 *   @param  none
 *   @return none
*/
TEST(testRobostness, computePathShouldBeRobost) {
    AStarAlgorithm aStar;
    vector<int> path1;
    vector<int> path2;

    aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);

    // set start, goal, and compute using A*
    aStar.PathFindingAlgorithm::setParam(1, 30);

    // assert that path can be generated
    ASSERT_TRUE(aStar.computPath(1.0));
    path1 = aStar.PathFindingAlgorithm::getPath();

    // run compute path for same start & goal for 100 times
    // and make sure results are consistent and no crash
    for (int i = 0; i < 1000; i++) {
        aStar.PathFindingAlgorithm::init(DEFAUTL_TEST_MAP);
        aStar.PathFindingAlgorithm::setParam(1, 30);

        ASSERT_TRUE(aStar.computPath(1.0));
        path2 = aStar.PathFindingAlgorithm::getPath();
        EXPECT_THAT(path1, ::testing::ContainerEq(path2));
    }
}

