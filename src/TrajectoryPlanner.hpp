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

public:
    
    TrajectoryPlanner();
    ~TrajectoryPlanner();
    void StraightXYVals(double car_x, double car_y, double car_yaw);
    void MakeTrajectory(double car_s, double car_d, double dest_s, double dest_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    double dist_inc;
    int steps;
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};

#endif /* TrajectoryPlanner_hpp */
