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



void TrajectoryPlanner::MakeTrajectory(double car_x, double car_s, double car_y, double car_v, double car_yaw, double dest_s, double dest_d, double dest_v, std::vector<double> prev_xpts, std::vector<double> prev_ypts){


    std::vector<double> spline_x_vals;
    std::vector<double> spline_y_vals;
    
    //define a pair of points at the start and end of the spline
    //otherwise the spline would be a straight line
    //dist_inc = (dest_s-car_s)/steps;

    //get global xy coords from s, d
  
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_angle = deg2rad(car_yaw);
  double ref_v = car_v;
  
    int prev_size = prev_xpts.size();
    double prev_car_x =0;
    double prev_car_y =0;
    
    //add starting points for trajectory calc
    if(prev_size<2){
      prev_car_x = car_x - cos(ref_angle);
      prev_car_y = car_y - sin(ref_angle);
    }
    //or continue previous path
    else{
      ref_x = prev_xpts[prev_size-1];
      ref_y = prev_ypts[prev_size-1];
      
      prev_car_x = prev_xpts[prev_size-2];
      prev_car_y = prev_ypts[prev_size-2];
      ref_angle = atan2(ref_y-prev_car_y,ref_x-prev_car_x);
      ref_v = distance(prev_car_x,prev_car_y,ref_x,ref_y)/PATH_INC_TIME;
    }
    spline_x_vals.push_back(prev_car_x);
    spline_x_vals.push_back(ref_x);
    spline_y_vals.push_back(prev_car_y);
    spline_y_vals.push_back(ref_y);
    
    
    //build end of spline
    auto next_wp0 = getXY(dest_s+10,dest_d,_maps_s,_maps_x,_maps_y);
    auto next_wp1 = getXY(dest_s+20,dest_d,_maps_s,_maps_x,_maps_y);
    auto next_wp2 = getXY(dest_s+40,dest_d,_maps_s,_maps_x,_maps_y);
    
    spline_x_vals.push_back(next_wp0[0]);
    spline_x_vals.push_back(next_wp1[0]);
    spline_x_vals.push_back(next_wp2[0]);
    
    spline_y_vals.push_back(next_wp0[1]);
    spline_y_vals.push_back(next_wp1[1]);
    spline_y_vals.push_back(next_wp2[1]);
    
    //convert to local coords
    for(int i=0; i<spline_x_vals.size(); i++){
      auto newPoint = XYGlobalToLocal(spline_x_vals[i],spline_y_vals[i],ref_x,ref_y,ref_angle);
      spline_x_vals[i] = newPoint[0];
      spline_y_vals[i] = newPoint[1];

    }
  
    tk::spline s;
    s.set_points(spline_x_vals,spline_y_vals);
    
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();
    
    //load existing path into next_x and next_y
    for(int i=0; i<prev_xpts.size();i++){
      next_x_vals.push_back(prev_xpts[i]);
      next_y_vals.push_back(prev_ypts[i]);
    }
    
    //calculate lengths
    double target_x = dest_s - car_s;
    double target_y = s(target_x);
    double target_dist = distance(0,0,target_x,target_y);
    
    double current_x = 0;
  
    for(int i= 1; i <= steps-prev_size; i++){
      double N = target_dist/(PATH_INC_TIME*ref_v);
      double x_point = current_x+target_x/N;
      double y_point = s(x_point);
      
      current_x = x_point;
      
      auto newPoint = XYLocalToGlobal(x_point,y_point,ref_x,ref_y,ref_angle);
      next_x_vals.push_back(newPoint[0]);
      next_y_vals.push_back(newPoint[1]);
      
      if(ref_v<dest_v)ref_v += .2;
      if(ref_v>dest_v)ref_v -= .2;
    }
}
 
  void TrajectoryPlanner::SetMaps(const std::vector<double> &maps_s,const std::vector<double> &maps_x,const std::vector<double> &maps_y){
    _maps_s = maps_s;
    _maps_x = maps_x;
    _maps_y = maps_y;
  }



