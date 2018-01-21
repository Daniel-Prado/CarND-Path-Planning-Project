#ifndef DP_ROAD_MAP_H
#define DP_ROAD_MAP_H

#include <string>
#include <vector>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include "spline.h"

using namespace std;

const double circuit_max_s = 6945.554;

class RoadMap {
public:
    //Constructor
    RoadMap(string map_file);
    
    //Destructor
    ~RoadMap();

    // Transform from Frenet s,d coordinates to Map coordinates X,Y based on splines that connect the track waypoints.
    vector<double> get_splined_xy(double s, double d);

    vector<double> get_frenet_vel(double s, double d, double speed, double car_to_map_yaw);

    static inline double safe_s(double s) {
        return ((s >= 0 && s < circuit_max_s) ? s : fmod(circuit_max_s + s, circuit_max_s));
    }

    static inline double safe_diff(double s1, double s2) {
        double a = safe_s(s1);
        double b = safe_s(s2);
        double diff = a - b;
        if(diff < -circuit_max_s / 2) // max_s/2 because that means half loop (closer from the other side)
            diff += circuit_max_s;
        if(diff > circuit_max_s / 2)
            diff -= circuit_max_s;
        return diff;
    }


    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

private:
    tk::spline spl_x;
    tk::spline spl_y;
    tk::spline spl_dx;
    tk::spline spl_dy;

    double endpoint_gap;

    double min_waypoint_s;
    double max_waypoint_s;
};

#endif