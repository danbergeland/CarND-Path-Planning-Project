//
//  PositionLogic.cpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#include "PositionLogic.hpp"
#include "geometryhelpers.hpp"
#define VERBOSE

double MAX_PLANNING_TIME = 4;
double LANE_WIDTH = 4;

//WEIGHTS
double weight_SPEED = 1;
double weight_COLLISION = 1;

PositionLogic::PositionLogic(){
    _TP = TrajectoryPlanner();
    _state = laneKeeping;
    _max_speed_mps = 22;
    _target_speed_mps = 21;
    SetAgression(.5);
    _vehicles = std::vector<std::vector<double>>();
    _target_d = 6;
}

PositionLogic::~PositionLogic(){
}

void PositionLogic::Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    _vehicles = vehicles;
    _car_d = car_d;
    _car_s = car_s;
    
    if(_state==laneKeeping){
        _TP.FollowLaneXYVals(_target_s, car_s, car_d, maps_s, maps_x, maps_y);
    }
}

void PositionLogic::checkTraffic(){
    
    double inLaneTarget = _target_s;
    std::vector<double> rightLaneCars;
    std::vector<double> leftLaneCars;

    for (int i = 0; i < _vehicles.size(); i++) {
        //s and d of nearby vehicles
        double s = _vehicles[i][5];
        double d = _vehicles[i][6];
        double vx = _vehicles[i][3];
        double vy = _vehicles[i][4];
        double lane_tol = 1;
        //check inlane forward
        if(_car_d < lane_tol+d && _car_d> d-lane_tol && s > _car_s){
            if(inLaneTarget > s-_car_s){
                inLaneTarget = s-_car_s-1;
            }
        }
        //check right lane
        else if (d > _car_d+lane_tol && d > _car_d+LANE_WIDTH+lane_tol){
            rightLaneCars.push_back(s);
        }
        //check left lane
        else if (d < _car_d-lane_tol && d < _car_d-LANE_WIDTH-lane_tol){
            leftLaneCars.push_back(s);
        }
    }
    
    double keepLaneCost = costPath(inLaneTarget, _car_d);
    
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
    _target_s = _target_speed_mps * _target_time;
}


double PositionLogic::costPath(double dest_s, double dest_d){
  double totalCost = 0;
  totalCost += costSpeed(dest_s);
  totalCost += costCollision(dest_s, dest_d);
  
  return totalCost;
}

double PositionLogic::costSpeed(double dest_s){
  return _agression*weight_SPEED*(fabs(dest_s-_target_s));
}

double PositionLogic::costCollision(double dest_s, double dest_d){
  double carPresent = 0;  
  for (int i = 0; i < _vehicles.size(); i++) {
    //s and d of nearby vehicles
    double s = _vehicles[i][5];
    double d = _vehicles[i][6];
    double vx = _vehicles[i][3];
    double vy = _vehicles[i][4];
    //Check rectangle formed by curent s/d and target s/d for vehicles
    if(s < dest_s-2 && s> _car_s-2){
      //car is in s range of path, so check lateral
      if(fabs(d-dest_d)<2 || fabs(d-_car_d) < 2){
        //car is in current lane, or target lane
        carPresent = 1;
      }
    }
  }
  return carPresent*(1-_agression)*weight_COLLISION;
}

std::vector<double> PositionLogic::NextXValues(){
    return _TP.next_x_vals;
}
std::vector<double> PositionLogic::NextYValues(){
    return _TP.next_y_vals;
    
}
