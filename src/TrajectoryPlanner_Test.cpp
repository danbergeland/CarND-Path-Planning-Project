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
    REQUIRE(TP.steps == 50);
}

TEST_CASE("UpdateXYVals returns x and y for each step", "TrajectoryPlanner"){
    TrajectoryPlanner TP = TrajectoryPlanner();
    REQUIRE(TP.next_x_vals.size()==0);
    REQUIRE(TP.next_y_vals.size()==0);
    TP.UpdateXYVals(0,0,0);
    REQUIRE(TP.next_x_vals.size()==TP.steps);
    REQUIRE(TP.next_y_vals.size()==TP.steps);
    
}
