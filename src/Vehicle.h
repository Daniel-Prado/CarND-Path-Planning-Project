#ifndef DP_VEHICLE_H
#define DP_VEHICLE_H


#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <iomanip>


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
    
    void       update_pos(double s, double d);
    void       update_vel(double s_dot, double d_dot);
    Vehicle    position_after_n_seconds(double incr_time);
    static int get_lane(const double d);
    int        get_lane() const;
    void       move_along_trajectory(const vector<double>& traj_s, const vector<double>& traj_d, size_t steps);


};

#endif
