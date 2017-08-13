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
    dist_inc = 0.44;
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

void TrajectoryPlanner::FollowLaneXYVals(double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    for(int i = 0; i < steps; i++)
    {
        auto nextXY = getXY(car_s+(dist_inc*i), car_d, maps_s, maps_x, maps_y);
        
        next_x_vals.push_back(nextXY[0]);
        next_y_vals.push_back(nextXY[1]);
    }
}

void TrajectoryPlanner::MakeTrajectory(double time_s, double car_s, double car_d, double dest_s, double dest_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    
    auto start_pos =getXY(car_s,car_d,maps_s,maps_x,maps_y);
    //planner can start at current x/y position; simulator can handle points in the trajectory behind the car
    next_x_vals.push_back(start_pos[0]);
    next_y_vals.push_back(start_pos[1]);
    
    //Calculate path in s / d coords
    
    //Convert s / d points on the path to x / y values
}

