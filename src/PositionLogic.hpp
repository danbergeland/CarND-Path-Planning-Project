//
//  PositionLogic.hpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#ifndef PositionLogic_hpp
#define PositionLogic_hpp


#include <stdio.h>
#include <vector>
#include "TrajectoryPlanner.hpp"

typedef enum {
    laneKeeping,
    changeLane,
} positionState;

class PositionLogic {
    positionState _state;
    TrajectoryPlanner _TP;
    double _agression;
    double _max_speed_mps;
    double _target_s;
    double _target_d;
    double _target_speed_mps;
    double _target_time;
    void checkTraffic(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d);
    
    
public:
    PositionLogic();
    ~PositionLogic();
    
    void Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    
    void SetAgression(double agression);
    
    std::vector<double> NextXValues();
    std::vector<double> NextYValues();
};



#endif /* PositionLogic_hpp */
