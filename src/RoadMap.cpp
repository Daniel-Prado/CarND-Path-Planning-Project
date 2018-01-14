#include "RoadMap.h"




RoadMap::RoadMap(string map_file) {

    ifstream in_map(map_file.c_str(), ifstream::in);

    string line;
    while (getline(in_map, line)) {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // The first point of the waypoints file and the last point are not the same, even if 
    // the track is circular. We will calculate the gap for later calculations.

    min_waypoint_s = map_waypoints_s[0];

    max_waypoint_s = map_waypoints_s[map_waypoints_s.size()-1];
    
    double endpoint_gap_x = map_waypoints_x[0] - map_waypoints_x[map_waypoints_x.size()-1];
    double endpoint_gap_y = map_waypoints_y[0] - map_waypoints_y[map_waypoints_y.size()-1];
    
    endpoint_gap = sqrt(endpoint_gap_x * endpoint_gap_x + endpoint_gap_y * endpoint_gap_y);

    // For spline calculations around the gap, we need some overlapping.
    vector<double> ext_x;
    vector<double> ext_y;
    vector<double> ext_s;
    vector<double> ext_dx;
    vector<double> ext_dy;

    double track_distance = max_waypoint_s - min_waypoint_s + endpoint_gap;
   
    int before_last_idx = map_waypoints_s.size() - 2;
    int last_idx = map_waypoints_s.size()-1;
        
    ext_x.push_back(map_waypoints_x[before_last_idx]);
    ext_y.push_back(map_waypoints_y[before_last_idx]);
    ext_s.push_back(map_waypoints_s[before_last_idx] - track_distance);
    ext_dx.push_back(map_waypoints_dx[before_last_idx]);
    ext_dy.push_back(map_waypoints_dy[before_last_idx]);
    ext_x.push_back(map_waypoints_x[last_idx]);
    ext_y.push_back(map_waypoints_y[last_idx]);
    ext_s.push_back(map_waypoints_s[last_idx] - track_distance);
    ext_dx.push_back(map_waypoints_dx[last_idx]);
    ext_dy.push_back(map_waypoints_dy[last_idx]);

    for(int i = 0; i < map_waypoints_s.size(); i++) {
        ext_x.push_back(map_waypoints_x[i]);
        ext_y.push_back(map_waypoints_y[i]);
        ext_s.push_back(map_waypoints_s[i]);
        ext_dx.push_back(map_waypoints_dx[i]);
        ext_dy.push_back(map_waypoints_dy[i]);
    }

    ext_x.push_back(map_waypoints_x[0]);
    ext_y.push_back(map_waypoints_y[0]);
    ext_s.push_back(map_waypoints_s[0] + track_distance);
    ext_dx.push_back(map_waypoints_dx[0]);
    ext_dy.push_back(map_waypoints_dy[0]);
    ext_x.push_back(map_waypoints_x[1]);
    ext_y.push_back(map_waypoints_y[1]);
    ext_s.push_back(map_waypoints_s[1] + track_distance);
    ext_dx.push_back(map_waypoints_dx[1]);
    ext_dy.push_back(map_waypoints_dy[1]);

    spl_x.set_points(ext_s,ext_x);
    spl_y.set_points(ext_s,ext_y);
    spl_dx.set_points(ext_s,ext_dx);
    spl_dy.set_points(ext_s,ext_dy);
}


RoadMap::~RoadMap() {}

// Transform from Frenet s,d coordinates to Map coordinates X,Y based on splines that connect the track waypoints.
vector<double> RoadMap::get_splined_xy(double s, double d) {
    
    double road_x = spl_x(s);
    double road_y = spl_y(s);
    
    double dx = spl_dx(s);
    double dy = spl_dy(s);
    
    double x = road_x + d * dx;
    double y = road_y + d * dy;
    
    return {x,y};
}
