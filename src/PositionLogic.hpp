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
    changeLaneLeft,
    changeLaneRight
} positionState;

typedef struct {
  double dest_s;
  double dest_d;
  double dest_v;
} target;

class PositionLogic {
    positionState _state;
    TrajectoryPlanner _TP;
    double _agression;
    double _max_speed_mps;
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
    
public:
    PositionLogic();
    ~PositionLogic();
    
    void Update(const std::vector<std::vector<double>> &vehicles, double car_s, double car_d, double car_speed, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    
    void SetAgression(double agression);
    
    std::vector<double> NextXValues();
    std::vector<double> NextYValues();
};


double getIdealLaneValue(double d);

#endif /* PositionLogic_hpp */
