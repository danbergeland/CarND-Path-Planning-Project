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
    void UpdateXYVals(double car_x, double car_y, double car_yaw);
    
    int steps;
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
};

#endif /* TrajectoryPlanner_hpp */
