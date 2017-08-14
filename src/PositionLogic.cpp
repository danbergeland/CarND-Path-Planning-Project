//
//  PositionLogic.cpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#include "PositionLogic.hpp"
#include "geometryhelpers.hpp"

double MAX_PLANNING_TIME = 4;

PositionLogic::PositionLogic(){
    _TP = TrajectoryPlanner::TrajectoryPlanner();
    _state = laneKeeping;
    SetAgression(.5);
}

PositionLogic::~PositionLogic(){
}

void PositionLogic::Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    
    
    
    if(_state==laneKeeping){
        _TP.FollowLaneXYVals(vehicles, car_s, car_d, maps_s, maps_x, maps_y);
    }
}

void PositionLogic::SetAgression(double agression){
    if(agression > .9){
        agression = .9;
    }
    else if(agression<0){
        agression = 0;
    }
    _agression = agression;
    _TP.setPlanTime((1-_agression)*MAX_PLANNING_TIME);
}

std::vector<double> PositionLogic::NextXValues(){
    return _TP.next_x_vals;
}
std::vector<double> PositionLogic::NextYValues(){
    return _TP.next_y_vals;
    
}
