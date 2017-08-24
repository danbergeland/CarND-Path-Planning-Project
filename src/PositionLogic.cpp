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
double weight_SPEED = 10;
double weight_COLLISION = 100;
double weight_ACCELERATION = 3;
double weight_TURN = 10;
double weight_PASSING_LANES = 3;

PositionLogic::PositionLogic(){
    _state = laneKeeping;
    _max_speed_mps = 22;
    //desired is the overall cruising speed based on agression
    _desired_speed_mps = 21;
    //target is the end velocity for the current path
    _target_speed_mps = 21;
    SetAgression(.7);
    _vehicles = std::vector<std::vector<double>>();
    _target_d = 6;
}

PositionLogic::~PositionLogic(){
}

vector<double> PositionLogic::Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, double car_speed){
    _vehicles = vehicles;
    _car_d = car_d;
    _car_s = car_s;
    //Convert car_speed mph to m/s
    _car_v = car_speed*.447;
    
    planPath();
    
    return {_target_s,_target_d,_target_speed_mps};
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
    
    //state machine
    if(_state == laneKeeping){
      if(_target_d != _car_d){
        _state = changeLane;
      }
    }
    if(_state == changeLane){
      if(fabs(_car_d-_target_d)<1){
        _state = laneKeeping;
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
    _target_time = 2.3;
    _desired_speed_mps = (_agression * _max_speed_mps)*.4 + _max_speed_mps*.7;
    _target_path_length = _desired_speed_mps * _target_time;
}


double PositionLogic::costPath(double dest_s, double dest_d, double dest_v){

  double cSpeed = costSpeed(dest_s);
  double cColl = costCollision(dest_s, dest_d);
  double cAcc = costAcceleration(dest_v);
  double cTurn = costTurn(dest_s, dest_d);
  double cLane = costPassingLane(dest_d);
/*  
  #ifdef VERBOSE
  std::cout<< std::printf("Costs: speed: %.1f, collision: %.1f, acceleration: %.1f, turn: %.1f \n", cSpeed,cColl,cAcc,cTurn);
  #endif
*/  
  return cSpeed+cColl+cAcc+cTurn+cLane;
}

double PositionLogic::costSpeed(double dest_s){

  return _agression*weight_SPEED*(fabs(dest_s-(_car_s+_target_path_length)));
}

double PositionLogic::costAcceleration(double dest_v){
  double accelCost = 0;
  
  accelCost = (1-_agression)*weight_ACCELERATION*(fabs(dest_v-_car_v));

  return accelCost;
}

double PositionLogic::costTurn(double dest_s, double dest_d){
  double turnCost = 0;
  if(fabs(dest_s-_car_s)>.1){
    turnCost = (1-_agression)*weight_TURN*(fabs(dest_d-_car_d)/fabs(dest_s-_car_s));
  }
  return turnCost;
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
    double speed = sqrt(vx*vx+vy*vy);
    if(s < dest_s && s+speed/3 > _car_s-5){
      //car is in s range of path, so check lateral
      if(fabs(d-dest_d)<2){
        carPresent = 1;
      }
    }
  }
  return carPresent*weight_COLLISION;
}

double PositionLogic::costPassingLane(double dest_d){
  //prefer the middle lane, and pass on left if possible
  double LaneCost = 0;
  if(dest_d>0 && dest_d<4){
    LaneCost = weight_PASSING_LANES;
  }
  if(dest_d>8){
    LaneCost = 2*weight_PASSING_LANES;
  }
  return LaneCost;
}

void PositionLogic::generateTargets(std::vector<target> &outTargetVector){
  if(_state==laneKeeping)
  {
    //Always attempt to go to each lane, at desired speed,
    target straight = {_car_s+_target_path_length,_car_d,_desired_speed_mps};
    outTargetVector.push_back(straight);
    
    //speed up
    target accel = {_car_s+_target_path_length,_car_d,_car_v+1};
    outTargetVector.push_back(accel);
    
    //slow down
    //target decel = {_car_s+_target_path_length,_car_d,_car_v-1};
    //outTargetVector.push_back(decel);

    
    if(_car_d > 4){
      target left = {_car_s+_target_path_length-4, _car_d-LANE_WIDTH,_desired_speed_mps};
      outTargetVector.push_back(left);
    }
    if(_car_d < 8){
      target right = {_car_s+_target_path_length-4, _car_d+LANE_WIDTH,_desired_speed_mps};
      outTargetVector.push_back(right);
    }

    for (int i = 0; i < _vehicles.size(); i++) {
      //s and d of nearby vehicles
      double s = _vehicles[i][5];
      double d = _vehicles[i][6];
      double vx = _vehicles[i][3];
      double vy = _vehicles[i][4];
      double speed = sqrt(vx*vx+vy*vy);
      //Check rectangle formed by curent s/d and target s/d for vehicles
      if(s < _car_s+_target_path_length && s > _car_s+2 && fabs(d-_car_d)<6){
        //add targets for each vehicle
        target followCar = {s, d, speed};
        outTargetVector.push_back(followCar);
      }
    }
    
  }
  //only make paths for target lane while changing lanes
  if(_state==changeLane){
    target finishLaneChange = {_car_s+_target_path_length,_target_d,_desired_speed_mps};
    outTargetVector.push_back(finishLaneChange);
    
    for (int i = 0; i < _vehicles.size(); i++) {
      //s and d of nearby vehicles
      double s = _vehicles[i][5];
      double d = _vehicles[i][6];
      double vx = _vehicles[i][3];
      double vy = _vehicles[i][4];
      double speed = sqrt(vx*vx+vy*vy);
      //Check rectangle formed by curent s/d and target s/d for vehicles
      if(s < _car_s+_target_path_length && s > _car_s-4 && fabs(d-_target_d)<1){
        //add targets for each vehicle
        target followCar = {s, _target_d, speed};
        outTargetVector.push_back(followCar);
      }
    }
  }
}

void PositionLogic::evaluateTargets(std::vector<target> &targetsVector){
  double lowest_cost = 999999;
  target bestTarget;
  for(int i =0; i<targetsVector.size(); i++){
    target thisTarget = targetsVector[i];
    double cost = costPath(thisTarget.dest_s,thisTarget.dest_d,thisTarget.dest_v);
    if(cost < lowest_cost){
      lowest_cost = cost;
      bestTarget = thisTarget;
    }
  }
  _target_s = bestTarget.dest_s;
  _target_d = getIdealLaneValue(bestTarget.dest_d);
  _target_speed_mps = bestTarget.dest_v;
  if(_target_speed_mps > .95* _max_speed_mps) _target_speed_mps = .95*_target_speed_mps;
}

double getIdealLaneValue(double d){
  for(int i=0; i<4; i++){
    if(d<i*LANE_WIDTH)
      return i*LANE_WIDTH-2;
  }
}


