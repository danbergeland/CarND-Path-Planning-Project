//
//  geometryhelpers.hpp
//  
//
//  Created by Dan Bergeland on 8/12/17.
//
//

#ifndef geometryhelpers_hpp
#define geometryhelpers_hpp


#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"



using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
void loadMapToVectors(vector<double> &map_waypoints_x, vector<double> &map_waypoints_y, vector<double> &map_waypoints_s, vector<double> &map_waypoints_dx, vector<double> &map_waypoints_dy, string mapFileName);
vector<double> XYGlobalToLocal(double xglobal, double yglobal,double car_x, double car_y, double car_yaw);
vector<double> XYLocalToGlobal(double xlocal, double ylocal,double car_x, double car_y, double car_yaw);
#endif /* geometryhelpers_hpp */
