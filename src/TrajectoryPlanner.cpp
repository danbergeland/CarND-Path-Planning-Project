//
//  TrajectoryPlanner.cpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#include "TrajectoryPlanner.hpp"
#include "geometryhelpers.hpp"
#include "spline.h"



TrajectoryPlanner::TrajectoryPlanner(){
    steps = 100;
    dist_inc = 0.44;
    max_speed_mps = 22;
    plan_time_s = 2;
    next_x_vals = vector<double>();
    next_y_vals = vector<double>();
}
TrajectoryPlanner::~TrajectoryPlanner(){
    
}

void TrajectoryPlanner::setPlanTime(double seconds){
    steps = (int)(seconds/PATH_INC_TIME);
    plan_time_s = seconds;
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

void TrajectoryPlanner::FollowLaneXYVals(double follow_dist, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    
    dist_inc = follow_dist/steps;
    
    for(int i = 0; i < steps; i++)
    {
        auto nextXY = getXY(car_s+(dist_inc*i),car_d , maps_s, maps_x, maps_y);
        next_x_vals.push_back(nextXY[0]);
        next_y_vals.push_back(nextXY[1]);
    }

}

void TrajectoryPlanner::setSpeedFollowVehicle(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d){

    double followDistance = steps * PATH_INC_TIME * max_speed_mps;
    
    for (int i = 0; i < vehicles.size(); i++) {
        double s = vehicles[i][5];
        double d = vehicles[i][6];
        double lane_tol = 1;
        if(car_d < lane_tol+d && car_d> d-lane_tol && s > car_s){
            if(followDistance>s-car_s){
                followDistance = s-car_s;
            }
        }
    }
    dist_inc = followDistance/steps;
}



void TrajectoryPlanner::MakeTrajectory(double car_s, double car_d, double car_v, double dest_s, double dest_d, double dest_v, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    
    auto start_pos =getXY(car_s,car_d,maps_s,maps_x,maps_y);
    //planner can start at current x/y position; simulator can handle points in the trajectory behind the car
    next_x_vals.push_back(start_pos[0]);
    next_y_vals.push_back(start_pos[1]);
    
    //Calculate path in s / d coords
    
    //Convert s / d points on the path to x / y values
}

