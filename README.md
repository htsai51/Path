# Path Finding Project
[![Build Status](https://travis-ci.org/htsai51/path.svg?branch=master)](https://travis-ci.org/htsai51/path)
[![Coverage Status](https://coveralls.io/repos/github/htsai51/path/badge.svg?branch=master)](https://coveralls.io/github/htsai51/path?branch=master)
---

## Overview

This is a simple global planner C++ project.  It assumes completely known environment 
and implements A Star algorithm to compute the shortest path using euclidean distance as 
heuristic estimation function.  It is designed so it could be modified/integrated to work in 
a robotic system to plan path under known environment (e.g. a warehouse).

Features:
* Shortest path finding using A Star algorithm
* Output shortest path on screen or to a file


This project chooses to implement A Star algorithm for path planning in a known environment due to its
efficiency and correctness.  Comparing to Dijkstra's algorithm which guarantees to find shortest path, 
A Star algorithm uses a heuristic estimation to searh for shortest path and is much faster than Dijkstra's 
algorithm.  Here is an example using the demo application:

- Running demo with default map.
- Select start node a 1 and goal node as 338.
- A Star computes the same cost (32) of shortest path as Dijkstra's algorithm but less steps are used (111 v.s. 254).

```bash
start is 1
goal is 338
Dijkstra's Shortest Path: 1 2 3 4 5 6 7 8 9 10 37 64 91 117 144 171 197 224 251 278 305 306 307 308 309 310 311 338
Dijkstra's total cost is 32
Dijkstra's total step is 254
A Star Shortest Path: 1 2 3 4 5 6 33 34 35 36 63 64 91 118 145 172 199 226 252 278 305 306 307 308 309 310 311 338
A Star total cost is 32
A Star total step is 111
```

This project also implements a simple unit test program which utilize googletest framework to 
perform tests on module function.  Test covers setting module parameters, computing shortest path,
correctiness of shortest path, and robustness of computing shortest path.

Finally, the implementation process of this project follows SIP model in software enginering.  Product backlog 
is cateogorized into three parts.  Each iteration is sub-tasked and each task is recorded with time 
or code change size (if applicable).

1. Implementation of Main Path Finding Algorithms
2. Implementation of Test Map, Demo, and Unit Tests
3. Code Optimization & Program Wrapup

Backlogs (product backlog, iteration logs, time logs) can be found at [BackLogs][reference-id-for-backlogs].

[reference-id-for-backlogs]: https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/11Ztb9IavDcHXACZNydmyZ2S24rh_3yXYII_1yvCACME/edit?usp=sharing



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

The main program takes in a csv file (if not specified, ../data/default.csv 
will be used) as known environment and finds a feasible shortest path given a start and a goal indices.

The input map is a NxM matrix with "1" indicating feasible path node and "O" indicating 
obstacle nodes that cannot be crossed.  It is assumed that there are 8 moving directions 
with unit movement cost except for diagonal movement which costs 1.5x more than straight 
movement (i.e. up, down, right, left).

In the demo, user can visualize the map with indices and choose start, goal indices for the program 
to compute the shortest path.  The output path can be displayed on screen or output to a 
CSV and text file as chosen by user.

- To start the program, in your ./build directory

```bash
./app/shell-app
```
- Follow onscreen instructions to run the demo
- Note that default map is storead at ../data/default.csv.  If you attemp to run demo
from a different root, please specify absolute path

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