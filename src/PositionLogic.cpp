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
double LANE_WIDTH = 4;

PositionLogic::PositionLogic(){
    _TP = TrajectoryPlanner::TrajectoryPlanner();
    _state = laneKeeping;
    _max_speed_mps = 22;
    _target_speed_mps = 21;
    SetAgression(.5);
    _target_s = _target_speed_mps * _target_time;
    _target_d = 6;
}

PositionLogic::~PositionLogic(){
}

void PositionLogic::Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    
    
    if(_state==laneKeeping){
        _TP.FollowLaneXYVals(_target_s, car_s, car_d, maps_s, maps_x, maps_y);
    }
}

void PositionLogic::checkTraffic(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d){
    
    double inLaneTarget = _target_s;
    std::vector<double> rightLaneCars;
    std::vector<double> leftLaneCars;

    for (int i = 0; i < vehicles.size(); i++) {
        //s and d of nearby vehicles
        double s = vehicles[i][5];
        double d = vehicles[i][6];
        double vx = vehicles[i][3];
        double vy = vehicles[i][4];
        double lane_tol = 1;
        //check inlane forward
        if(car_d < lane_tol+d && car_d> d-lane_tol && s > car_s){
            if(inLaneTarget > s-car_s){
                inLaneTarget = s-car_s;
            }
        }
        //check right lane
        else if (d > car_d+lane_tol && d > car_d+LANE_WIDTH+lane_tol){
            rightLaneCars.push_back(s);
        }
        //check left lane
        else if (d < car_d-lane_tol && d < car_d-LANE_WIDTH-lane_tol){
            leftLaneCars.push_back(s);
        }
    }
    
    if (_state==laneKeeping) {
        if(inLaneTarget < .9 * _target_s){
            _state=changeLane;
        }
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
    _target_time = (1-_agression)*MAX_PLANNING_TIME;
    _TP.setPlanTime(_target_time);
}

std::vector<double> PositionLogic::NextXValues(){
    return _TP.next_x_vals;
}
std::vector<double> PositionLogic::NextYValues(){
    return _TP.next_y_vals;
    
}
