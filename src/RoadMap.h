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

class RoadMap {
public:
    //Constructor
    RoadMap(string map_file);
    
    //Destructor
    ~RoadMap();

    // Transform from Frenet s,d coordinates to Map coordinates X,Y based on splines that connect the track waypoints.
    vector<double> get_splined_xy(double s, double d);


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