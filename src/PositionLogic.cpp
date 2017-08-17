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
double weight_COLLISION = 50;
double weight_ACCELERATION = 5;
double weight_TURN = 10;

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

void PositionLogic::Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, double car_speed, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y){
    _vehicles = vehicles;
    _car_d = car_d;
    _car_s = car_s;
    _car_v = car_speed;
    
    planPath();
    
    if(_state==laneKeeping){
        _TP.FollowLaneXYVals(_target_path_length, car_s, car_d, maps_s, maps_x, maps_y);
    }
}

void PositionLogic::planPath(){
    
    double inLaneTarget = _target_path_length;
    std::vector<double> rightLaneCars;
    std::vector<double> leftLaneCars;
    
    std::vector<target> targetPositions;
    //generate target s, d and v
    generateTargets(targetPositions);
    //evaluate cost of each traj
    evaluateTargets(targetPositions);
    //give the lowest cost traj to the trajectoryPlanner
    //_TP.MakeTrajectory(_car_s,_car_d,_target_s,_target_d,)
    _target_s = _car_s + _target_path_length;
    double keepLaneCost = costPath(_target_s, _car_d, _car_v);
    
    std::cout<< keepLaneCost << "\n";
    
}

void PositionLogic::SetAgression(double agression){
    if(agression > .9){
        agression = .9;
    }
    else if(agression<0){
        agression = 0;
    }
    _agression = agression;
    _target_time = (1-_agression*.75)*MAX_PLANNING_TIME;
    _target_speed_mps = (_agression * _max_speed_mps)*.29 + _max_speed_mps*.7;
    _TP.setPlanTime(_target_time);
    _target_path_length = _target_speed_mps * _target_time;
}


double PositionLogic::costPath(double dest_s, double dest_d, double dest_v){
  double totalCost = 0;
  totalCost += costSpeed(dest_s);
  totalCost += costCollision(dest_s, dest_d);
  totalCost += costAcceleration(dest_v);
  totalCost += costTurn(dest_s, dest_d);
  return totalCost;
}

double PositionLogic::costSpeed(double dest_s){
  return _agression*weight_SPEED*(fabs(dest_s-(_car_s+_target_path_length)));
}

double PositionLogic::costAcceleration(double dest_v){
  return (1-_agression)*weight_ACCELERATION*(fabs(dest_v-_car_v));
}

double PositionLogic::costTurn(double dest_s, double dest_d){
  if(fabs(dest_s-_car_s)>.1){
    return (1-_agression)*weight_TURN*(fabs(dest_d-_car_d)/fabs(dest_s-_car_s));
  }
  return 0;
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
    if(s < dest_s && s > _car_s-2){
      //car is in s range of path, so check lateral
      if(fabs(d-dest_d)<2){
        carPresent = 1;
      }
    }
  }
  return carPresent*(1-_agression)*weight_COLLISION;
}

void PositionLogic::generateTargets(std::vector<target> &outTargetVector){
  
  //Always attempt to go straight
  target straight = {_car_s+_target_path_length,_car_d,_car_v};
  outTargetVector.push_back(straight);
  
  //Always add stop short option
  target stop = {_car_s+10,_car_d, 0};
  
  for (int i = 0; i < _vehicles.size(); i++) {
    //s and d of nearby vehicles
    double s = _vehicles[i][5];
    double d = _vehicles[i][6];
    double vx = _vehicles[i][3];
    double vy = _vehicles[i][4];
    double speed = sqrt(vx*vx+vy*vy);
    //Check rectangle formed by curent s/d and target s/d for vehicles
    if(s < _car_s+_target_path_length && s > _car_s-4){
      //add targets for each vehicle
      target followCar = {s-2, d, speed};
      outTargetVector.push_back(followCar);
    }
  }
}

void PositionLogic::evaluateTargets(std::vector<target> &targetsVector){
  
}

std::vector<double> PositionLogic::NextXValues(){
    return _TP.next_x_vals;
}
std::vector<double> PositionLogic::NextYValues(){
    return _TP.next_y_vals;
    
}
