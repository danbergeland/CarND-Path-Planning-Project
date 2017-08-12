//
//  TrajectoryPlanner.cpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#include "TrajectoryPlanner.hpp"
#include "geometryhelpers.hpp"



TrajectoryPlanner::TrajectoryPlanner(){
    steps = 50;
    dist_inc = 0.5;
    next_x_vals = vector<double>();
    next_y_vals = vector<double>();
}
TrajectoryPlanner::~TrajectoryPlanner(){
    
}

void TrajectoryPlanner::StraightXYVals(double car_x, double car_y, double car_yaw)
{
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    for(int i = 0; i < steps; i++)
    {
        next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
    }
}

void TrajectoryPlanner::MakeTrajectory(double car_s, double car_d, double dest_s, double dest_d){
    
}

