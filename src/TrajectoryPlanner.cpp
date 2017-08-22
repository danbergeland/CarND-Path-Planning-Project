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



void TrajectoryPlanner::MakeTrajectory(double car_x, double car_y, double car_v, double car_yaw, double dest_s, double dest_d, double dest_v, std::vector<double> prev_xpts, std::vector<double> prev_ypts){
    next_y_vals = std::vector<double>();
    next_x_vals = std::vector<double>();

    std::vector<double> spline_x_vals;
    std::vector<double> spline_y_vals;
    
    //define a pair of points at the start and end of the spline
    //otherwise the spline would be a straight line
    //dist_inc = (dest_s-car_s)/steps;

    //get global xy coords from s, d
    
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_angle = deg2rad(car_yaw);
    
//    auto start_pos1={ref_x-cos(ref_angle),car_y};
//    auto position = getXY(car_s,car_d,maps_s,maps_x,maps_y);
//    auto start_pos2=getXY(car_s+dist_inc/2,car_d,maps_s,maps_x,maps_y);
//    auto end_pos1 = getXY(dest_s-dist_inc,dest_d,maps_s,maps_x,maps_y);
//    auto end_pos2 = getXY(dest_s, dest_d,maps_s,maps_x,maps_y);
//    std::cout << "global start : "<< start_pos1[0]<<", "<<start_pos1[1]<<" end: "<<end_pos2[0]<<", "<<end_pos2[1]<<"\n";
//    
//    //convert to local
//    auto local_start1 = XYGlobalToLocal(start_pos1[0],start_pos1[1],position[0],position[1],car_yaw);
//    auto local_start2 = XYGlobalToLocal(start_pos2[0],start_pos2[1],position[0],position[1],car_yaw);
//    auto local_end1 = XYGlobalToLocal(end_pos1[0],end_pos1[1],position[0],position[1],car_yaw);
//    auto local_end2 = XYGlobalToLocal(end_pos2[0],end_pos2[1],position[0],position[1],car_yaw);
//    std::cout << "local start : "<< local_start1[0]<<", "<<local_start1[1]<<" end: "<<local_end2[0]<<", "<<local_end2[1]<<"\n";
//    
//    //spline with local
//    tk::spline s;
//    spline_x_vals.push_back(local_start1[0]);
//    spline_x_vals.push_back(local_start2[0]);
//    spline_x_vals.push_back(local_end1[0]);
//    spline_x_vals.push_back(local_end2[0]);
//    
//    spline_y_vals.push_back(local_start1[1]);
//    spline_y_vals.push_back(local_start2[1]);
//    spline_y_vals.push_back(local_end1[1]);
//    spline_y_vals.push_back(local_end2[1]);
//    
//    
//    s.set_points(spline_x_vals,spline_y_vals);
//    //make points
//    for(int i=0; i<steps;i++){
//      double local_x = i*dist_inc;
//      auto XYposition = XYLocalToGlobal(local_x,s(local_x),position[0],position[1],car_yaw);
//      next_x_vals.push_back(XYposition[0]);
//      next_y_vals.push_back(XYposition[1]);
//    }
//  
    
    
//      spline_x_vals.push_back(car_s);
//      spline_x_vals.push_back(car_s+dist_inc);
//      spline_x_vals.push_back(dest_s-dist_inc);
//      spline_x_vals.push_back(dest_s);
//      
//      spline_y_vals.push_back(car_d);
//      spline_y_vals.push_back(car_d);
//      spline_y_vals.push_back(dest_d);
//      spline_y_vals.push_back(dest_d);
//
//      tk::spline s;
//      s.set_points(spline_x_vals,spline_y_vals);
//      //Calculate path in s / d coords
//      double current_s = 0;
//      std::vector<double> spline_s_vals;
//      std::vector<double> spline_d_vals;
//
//      for(int i=0; i<steps;i++){
//        double s_val = current_s+dist_inc+car_s;
//        spline_s_vals.push_back(s_val);
//        spline_d_vals.push_back(s(s_val));
//        
//        current_s += dist_inc;
//      }
//      
//      //Convert s / d points on the path to x / y values
//      for(int i=0; i<steps; i++){
//        auto nextXY = getXY(spline_s_vals[i],spline_d_vals[i], maps_s, maps_x, maps_y);
//        next_x_vals.push_back(nextXY[0]);
//        next_y_vals.push_back(nextXY[1]);
//      }
    
}
 
  void TrajectoryPlanner::SetMaps(const std::vector<double> &maps_s,const std::vector<double> &maps_x,const std::vector<double> &maps_y){
    _maps_s = maps_s;
    _maps_x = maps_x;
    _maps_y = maps_y;
  }



