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

#define PATH_INC_TIME .02

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

    if(car_s < dest_s){

      std::vector<double> spline_x_vals;
      std::vector<double> spline_y_vals;
      
      //define a pair of points at the start and end of the spline
      //otherwise the spline would be a straight line
      dist_inc = (dest_s-car_s)/steps;
      //start_dist_inc = dist_inc;
      spline_x_vals.push_back(car_s);
      spline_x_vals.push_back(car_s+dist_inc);
      //end_dist_inc = dist_inc;
      spline_x_vals.push_back(dest_s-dist_inc);
      spline_x_vals.push_back(dest_s);
      
      spline_y_vals.push_back(car_d);
      spline_y_vals.push_back(car_d);
      spline_y_vals.push_back(dest_d);
      spline_y_vals.push_back(dest_d);

      tk::spline s;
      s.set_points(spline_x_vals,spline_y_vals);
      //Calculate path in s / d coords
      double current_s = 0;
      std::vector<double> spline_s_vals;
      std::vector<double> spline_d_vals;
      
      for(int i=0; i<steps;i++){
        double s_val = current_s+dist_inc+car_s;
        spline_s_vals.push_back(s_val);
        spline_d_vals.push_back(s(s_val));
        
        current_s += dist_inc;
      }
      
      //Convert s / d points on the path to x / y values
      for(int i=0; i<steps; i++){
        auto nextXY = getXY(spline_s_vals[i],spline_d_vals[i], maps_s, maps_x, maps_y);
        next_x_vals.push_back(nextXY[0]);
        next_y_vals.push_back(nextXY[1]);
      }
      
    }
}


