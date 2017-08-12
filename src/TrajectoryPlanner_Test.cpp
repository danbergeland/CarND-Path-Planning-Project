//
//  TrajectoryPlanner_Test.cpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#define CATCH_CONFIG_MAIN

#include <stdio.h>
#include <vector>

#include "../Catch/single_include/catch.hpp"

#include "TrajectoryPlanner.hpp"



TEST_CASE("Test Framework Works", "TrajectoryPlanner"){
    REQUIRE(true);
}

TEST_CASE("Create a TrajectoryPlanner object", "TrajectoryPlanner"){
    TrajectoryPlanner TP = TrajectoryPlanner();
    REQUIRE(TP.Steps == 0);
}
