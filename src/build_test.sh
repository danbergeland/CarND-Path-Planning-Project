#! /bin/bash
g++ -std=c++11 -I../Catch/include -I/usr/local/include  TrajectoryPlanner_Test.cpp geometryhelpers.cpp TrajectoryPlanner.cpp -o run_test
./run_test
