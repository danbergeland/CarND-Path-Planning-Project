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
    changeLane
} positionState;

typedef struct {
  double dest_s;
  double dest_d;
  double dest_v;
} target;

class PositionLogic {
    positionState _state;
    double _agression;
    double _max_speed_mps;
    double _desired_speed_mps;
    double _target_path_length;
    double _target_d;
    double _target_s;
    double _target_speed_mps;
    double _target_time;
    double _car_s;
    double _car_d;
    double _car_v;
    std::vector<std::vector<double>> _vehicles;
    
    void planPath();
    void generateTargets(std::vector<target> &outTargetVector);
    void evaluateTargets(std::vector<target> &targetsVector);
    double costPath(double dest_s, double dest_d, double dest_v);
    double costSpeed(double dest_s);
    double costAcceleration(double dest_v);
    double costCollision(double dest_s, double dest_d);
    double costTurn(double dest_s, double dest_d);
    double costPassingLane(double dest_d);
    
public:
    PositionLogic();
    ~PositionLogic();
    
    std::vector<double> Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, double car_speed);
    
    void SetAgression(double agression);
};


double getIdealLaneValue(double d);

#endif /* PositionLogic_hpp */
