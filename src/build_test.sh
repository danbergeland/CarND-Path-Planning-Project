#! /bin/bash
g++ -I../Catch/include TrajectoryPlanner_Test.cpp TrajectoryPlanner.cpp -o run_test
./run_test --success
