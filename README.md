# Path Finding Project
[![Build Status](https://travis-ci.org/htsai51/path.svg?branch=master)](https://travis-ci.org/htsai51/path)
[![Coverage Status](https://coveralls.io/repos/github/htsai51/path/badge.svg?branch=master)](https://coveralls.io/github/htsai51/path?branch=master)
---

## Overview

This is a simple global planner C++ project.  


The purpose of this project is to design a path planning module that can be integrated to work in 
a robotic system to navigate autonomously in a known environment (e.g. a warehouse).  A completely 
known environment is assumed in this project and A Star algorithm is implemented to compute the 
shortest path using euclidean distance as heuristic estimation function.  


Feature:
* Shortest path finding using A Star algorithm


This project chose to implement A Star algorithm for path planning in a known environment due to its
efficiency and correctness.  Comparing to Dijkstra's algorithm which guarantees to find shortest path, 
A Star algorithm uses a heuristic estimation to searh for shortest path and is much faster than Dijkstra's 
algorithm.  Here is an example of performance comparison of A Star and Dijkstra's algorithm:

- Running demo with default map.
- Select start node a 144 and goal node as 316.
- A Star computed the same cost (16) of shortest path as Dijkstra's algorithm but used significantly less time.

```bash
Please enter start, goal indices
144
316

Dijkstra's Shortest Path: 144 171 197 222 221 246 271 296 295 294 293 292 291 316
Dijkstra's total cost is 16
Dijkstra's search time is 0.023648 seconds

A Star Shortest Path: 144 171 197 222 221 246 271 296 295 294 293 292 291 316
A Star total cost is 16
A Star search time is 0.00816 seconds
```


This project also implements a simple unit test program which utilize googletest framework to 
perform tests on module function.  Test covers setting module parameters, computing shortest path,
correctiness of shortest path, and robustness of computing shortest path.


Finally, the implementation process of this project follows SIP model in software enginering.  Product backlog 
is cateogorized into three parts.  Each iteration is sub-tasked and each task is recorded with time 
or code commit size (if applicable).

1. Implementation of Main Path Finding Algorithms
2. Implementation of Test Map, Demo, and Unit Tests
3. Code Optimization & Program Wrapup

(To include estimation time v.s. total cost time)??

Backlogs (product backlog, iteration logs, time logs) can be found at [BackLogs][reference-id-for-backlogs].

[reference-id-for-backlogs]: https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/11Ztb9IavDcHXACZNydmyZ2S24rh_3yXYII_1yvCACME/edit?usp=sharing

UML class/activity diagrams can be found in git under ./UML folder.



## License

This program is under MIT License.

Copyright (c) 2017 Huei-Tzu Tsai
```bash
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
```


## Dependencies

None


## How to build

- Checkout the repo (and submodules)
```bash
$ git clone --recursive https://github.com/htsai51/path.git
cd path
mkdir -p build
cd build
cmake ..
make
```


## How to run demo

The main program takes in a csv file (press ctl+d to use default) as 
known environment and finds a feasible shortest path given a start and a goal indices.

The input map is a NxM matrix with "1" indicating free node and "O" indicating 
obstacle nodes that cannot be crossed.  It is assumed that there are 8 moving directions 
with unit movement cost except for diagonal movement which costs 1.5x more than horizontal/vertical  
movement (i.e. up, down, right, left).  User can supply his/her own map using the rules specified.

In the demo, user can visualize the map with indices  (please maximize console window) and 
choose start, goal indices for the program to compute the shortest path.  The output path can 
be displayed on screen or output to a CSV and text file as chosen by user.

- To start the program, in your ./build directory

```bash
./app/shell-app
```

- Follow onscreen instructions to run the demo
- Note that default map is stored at ../data/default.csv.  If you attempt to run demo
from a different root, please specify absolute path
- The resulting shortest path indices will be outputed on screen.
- More options can be found to output path as:
    * 0: View path in map on screen
    * 1: Save path into csv (path in map at ../data/out.csv) and text file (path only at ../data/path.txt)
    * 2: Both of above


## How to run unit tests

- In your ./build directory

```bash
./test/cpp-test
```


## How to generate doxygen documentation

- In your . directory

```bash
doxygen ./Doxygen
```

- Doxygen files will be generated to ./docs folder