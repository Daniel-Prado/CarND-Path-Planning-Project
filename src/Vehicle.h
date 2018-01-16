#ifndef DP_VEHICLE_H
#define DP_VEHICLE_H


#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>

using namespace std;

class Vehicle {

public:
    int id;
    double s;
    double s_dot;
    double s_ddot;
    double d;
    double d_dot;
    double d_ddot;

    Vehicle(const int identifier);
    
    void       updatePos(double s, double d);
    void       updateVelocity(double s_dot, double d_dot);
    Vehicle    position_after_n_seconds(double incr_time);
    static int get_lane(const double d);
    int        getLane();
    void       move_along_trajectory(const vector<double>& traj_s, const vector<double>& traj_d, int steps);


};

#endif
