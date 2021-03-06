//
//  TrajectoryPlanner.hpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#ifndef TrajectoryPlanner_hpp
#define TrajectoryPlanner_hpp


#include <stdio.h>
#include <vector>

class TrajectoryPlanner{
    void setSpeedFollowVehicle(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d);
    double max_speed_mps;
    double dist_inc;
    int steps;
    double plan_time_s;
    std::vector<double> _maps_s;
    std::vector<double> _maps_x;
    std::vector<double> _maps_y;

public:
    
    TrajectoryPlanner();
    ~TrajectoryPlanner();
    
    void setPlanTime(double seconds);
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    
    void StraightXYVals(double car_x, double car_y, double car_yaw);
    void FollowLaneXYVals(double follow_dist, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    void MakeTrajectory(double car_x, double car_s, double car_y, double car_v, double car_yaw, double dest_s, double dest_d, double dest_v, std::vector<double> prev_xpts, std::vector<double> prev_ypts);
    void SetMaps(const std::vector<double> &maps_s, const std::vector<double> &maps_x,const std::vector<double> &maps_y);

};

#endif /* TrajectoryPlanner_hpp */
