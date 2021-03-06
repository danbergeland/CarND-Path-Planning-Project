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

#include "geometryhelpers.hpp"
#include "TrajectoryPlanner.hpp"



TEST_CASE("Test Framework Works", "TrajectoryPlanner"){
    REQUIRE(true);
}

TEST_CASE("Create a TrajectoryPlanner object, check steps default to 50", "TrajectoryPlanner"){
    TrajectoryPlanner TP = TrajectoryPlanner();
    REQUIRE(TP.steps == 50);
}

TEST_CASE("UpdateXYVals returns x and y for each step", "TrajectoryPlanner"){
    TrajectoryPlanner TP = TrajectoryPlanner();
    REQUIRE(TP.next_x_vals.size()==0);
    REQUIRE(TP.next_y_vals.size()==0);
    TP.StraightXYVals(0,0,0);
    REQUIRE(TP.next_x_vals.size()==TP.steps);
    REQUIRE(TP.next_y_vals.size()==TP.steps);
    
}

TEST_CASE("Check Geometry of straight line", "TrajectoryPlanner"){
    TrajectoryPlanner TP = TrajectoryPlanner();
    TP.steps = 5;
    TP.StraightXYVals(0,0,0);
    REQUIRE(TP.next_x_vals[1] == TP.dist_inc);
    REQUIRE(TP.next_x_vals[4] == TP.dist_inc*4);
    REQUIRE(TP.next_y_vals[4] == 0);
}

TEST_CASE("Check maps are loaded to correct number of entries", "GeometryHelpers"){
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
    loadMapToVectors(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, "../data/highway_map.csv");
    
    REQUIRE(map_waypoints_x.size() ==181);
    REQUIRE(map_waypoints_y.size() ==181);
    REQUIRE(map_waypoints_s.size() ==181);
    REQUIRE(map_waypoints_dx.size() ==181);
    REQUIRE(map_waypoints_dy.size() ==181);

}



