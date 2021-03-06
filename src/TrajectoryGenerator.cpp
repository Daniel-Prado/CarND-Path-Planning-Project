#include "TrajectoryGenerator.h"


TrajectoryGenerator::~TrajectoryGenerator() {
}

void TrajectoryGenerator::update_previous_path(size_t prev_path_length) {

    size_t consumed_path = previous_path_s.size() - prev_path_length;
    
    // The previous_path_x provided by the simulator contains the points not yet visited by the car (prev_path_length)
    // however our 'locally' stored previous_path_s and _d stores the total 50 points,
    // so we need to delete the points in the past.
    if(prev_path_length > 0 && consumed_path > 0) {

        previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin() + consumed_path);
        previous_path_d.erase(previous_path_d.begin(), previous_path_d.begin() + consumed_path);

    // In the extraordinary rare case that the car has visited ALL the points in the previous path,
    // we just get rid of our local copy (it's past, so it's useless now.)
    // In practice this will not happen because the car has only time to visit 2 or 3 points 
    } else if(prev_path_length == 0) {

        previous_path_s.clear();
        previous_path_d.clear();
    }
}

vector<vector<double>> TrajectoryGenerator::get_previous_trajectory() {
    return {previous_path_s, previous_path_d};
}

size_t TrajectoryGenerator::get_length() {
    return previous_path_s.size();
}

vector<vector<double>> TrajectoryGenerator::generate_new_trajectory(vector<Vehicle>& prev_trajectory, Vehicle& car_at_start, Vehicle& car_at_goal, const int n_steps) {

    // Create a list of widely spaced (s,d) waypoints, evenly spaced at 30m.
    // Later we will interpolate waypoints with a spline and fill it in with more points 
    vector<double> pts_s;
    vector<double> pts_d;

    // Reference s,d
    // either we will reference the starting point as where the car is or at the previous path end point.
    double ref_s = car_at_start.s;
    double ref_d = car_at_start.d;

    int prev_size = prev_trajectory.size();

    // if previous path is almost empty, use the car as starting reference.
    if (prev_size < 2) {
        // use two points that nake the path tangent to the car.
        // This will be executed only at the start.
        double prev_car_s = ref_s - 1;
        double prev_car_d = ref_d;

        pts_s.push_back(prev_car_s);
        pts_s.push_back(ref_s);

        pts_d.push_back(prev_car_d);
        pts_d.push_back(ref_d);
    }
    else {
        // use the previous path end point as a starting reference
        double ref_s_prev = prev_trajectory[prev_size-2].s;
        double ref_d_prev = prev_trajectory[prev_size-2].d;

        if (ref_s_prev > ref_s + circuit_max_s/2)
            // In this case ref_s > 0 but ref_s_prev is close to circuit_max_s
            ref_s_prev -= circuit_max_s; // it will be negative

        // use two points that make the path tangent to the previous path's end point slope.
        pts_s.push_back(ref_s_prev);
        pts_s.push_back(ref_s);

        pts_d.push_back(ref_d_prev);
        pts_d.push_back(ref_d);

    }

    // The 3rd point will be the car at goal desired position.
    double s_goal = car_at_goal.s;
    if (s_goal < (car_at_start.s - circuit_max_s/2))
        s_goal += circuit_max_s;

    //double third_point = max (s_goal, pts_s.back()+30);
    double third_point = pts_s.back()+45;

    pts_s.push_back(third_point);
    pts_d.push_back(car_at_goal.d);

    // 4th and 5th points will be in the same d position as the car_at_goal, 30m away each in s.
    pts_s.push_back(third_point + 15.0);
    pts_d.push_back(car_at_goal.d);
    pts_s.push_back(third_point + 45.0);
    pts_d.push_back(car_at_goal.d);

    // Create a spline
    tk::spline spl;
    /*if(car_at_start.s > 6850) {
    /cout << "SPLINE pts_s: ";
    
        for (auto &titi: pts_s)
        {
                cout << titi << " ";
        }
        cout << endl;
    }*/

    spl.set_points(pts_s, pts_d);

    // Construct trajectory from car_at_start till car_at_goal along the spline.
    vector<double> next_s_vals;
    vector<double> next_d_vals;

    // We calculate how to break up spline points so that we travel at our desired reference velocity.
    // NOTE this assumes constant velocity, even if the ref_vel could be accelerating or decelerating even at
    // 10m/s2 . Maybe the code can be improved taking this into account, so that the points will not be evenly spaced.

    double t = 0.02 * n_steps;
    double acc = 2 * RoadMap::safe_diff(RoadMap::safe_diff(car_at_goal.s, car_at_start.s) , car_at_start.s_dot * t) / (t*t);

    for(int i = 1; i <= n_steps; i++) {
        t = i * 0.02;
        double s_spline = ref_s + (car_at_start.s_dot * t) + (0.5 * acc * t*t);
        double d_spline = spl(s_spline);
        s_spline = RoadMap::safe_s(s_spline);

        next_s_vals.push_back(s_spline);
        next_d_vals.push_back(d_spline);
    }

    return {next_s_vals, next_d_vals};
}

vector<vector<double>> TrajectoryGenerator::update_trajectory_for_plan(const vector<Vehicle>& plan) {
  
    previous_path_s.clear();
    previous_path_d.clear();

    for(const Vehicle& veh_at_step : plan) {
        previous_path_s.push_back(veh_at_step.s);
        previous_path_d.push_back(veh_at_step.d);
    }

    // Convert to map coordinates.
    vector<double> next_path_x;
    vector<double> next_path_y;

    for(int i = 0; i < previous_path_s.size(); i++) {
        const vector<double> &next_point = this->_road.get_splined_xy(previous_path_s[i], previous_path_d[i]);

        next_path_x.push_back(next_point[0]);
        next_path_y.push_back(next_point[1]);
    }

    return {next_path_x, next_path_y};
}
/*
vector<vector<double>> TrajectoryGenerator::get_new_frenet_trajectory(int prev_size, double ref_vel, int lane, double car_s, double car_d) {

    // Create a list of widely spaced (s,d) waypoints, evenly spaced at 30m.
    // Later we will interpolate waypoints with a spline and fill it in with more points 
    vector<double> pts_s;
    vector<double> pts_d;

    // Reference s,d
    // either we will reference the starting point as where the car is or at the previous path end point.
    double ref_s = car_s;
    double ref_d = car_d;
    //double ref_yaw = deg2rad(car_yaw);

    // if previous path is almost empty, use the car as starting reference.
    cout << "prev_size: " << prev_size << endl;
    if (prev_size < 2) {
        cout << "WARNING: prev_size < 2" << endl;
        // use two points that nake the path tangent to the car.
        // This will be executed only at the start.
        double prev_car_s = car_s - 1;
        double prev_car_d = car_d;

        pts_s.push_back(prev_car_s);
        pts_s.push_back(car_s);

        pts_d.push_back(prev_car_d);
        pts_d.push_back(car_d);
    }
    else {
        // use the previous path end point as a starting reference
        cout << "previous_path_s (inversa): " << endl;
        for (int i = prev_size-1; i >=0 ; i--)
        {
            cout << previous_path_s[i] << "  ";
        }
        cout << endl;

        ref_s = previous_path_s[prev_size-1];
        ref_d = previous_path_d[prev_size-1];

        double ref_s_prev = previous_path_s[prev_size-2];
        double ref_d_prev = previous_path_d[prev_size-2];

        //ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // use two points that make the path tangent to the previous path's end point slope.
        pts_s.push_back(ref_s_prev);
        pts_s.push_back(ref_s);

        pts_d.push_back(ref_d_prev);
        pts_d.push_back(ref_d);

    }

    // So far we have calculated just the first two points in the list, to ensure that the spline
    // will be tangent to the latest previous_path at the endpoint.
    // Now we need to add more points.

    // In Frenet, add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp30m = { car_s + 30.0, double(2+4*lane) };
    vector<double> next_wp60m = { car_s + 60.0, double(2+4*lane) };
    vector<double> next_wp90m = { car_s + 90.0, double(2+4*lane) };

    pts_s.push_back(next_wp30m[0]);
    pts_s.push_back(next_wp60m[0]);
    pts_s.push_back(next_wp90m[0]);

    pts_d.push_back(next_wp30m[1]);
    pts_d.push_back(next_wp60m[1]);
    pts_d.push_back(next_wp90m[1]);

    // Create a spline
    tk::spline spl;

    for (int i = 0; i < pts_s.size() ; i++) {
        cout << "(s,d)[" << i << "]: " << pts_s[i] << " " << pts_d[i] << "; " ;
    }
    cout << endl;

    spl.set_points(pts_s, pts_d);

    // Now we are going to build the next_s_vals and next_d_vals that we actually will send to the simulator
    // To make the trajectory changes smooth, we are re-using the part of the previous path trajectory that the
    // car hasn't made yet... For example of a 50 bullet points trajectory, the car maybe has only reached the\
    // 3rd one, so 47 bullets are still reusable.
    // NOTE to myself: This may not work if sudden changes in the trajectory may be required (to avoid an accident for example)
    vector<double> next_s_vals;
    vector<double> next_d_vals;

    cout << "before next_s_vals -- previous_path_s.size: " << previous_path_s.size() << endl;
    for (int i = 0; i < prev_size; i++) {
        cout << "pp_s[" <<i <<"]" << previous_path_s[i] << " ";
        next_s_vals.push_back(previous_path_s[i]);
        next_d_vals.push_back(previous_path_d[i]);

    }

    // We calculate how to break up spline points so that we travel at our desired reference velocity.
    // NOTE this assumes constant velocity, even if the ref_vel could be accelerating or decelerating even at
    // 10m/s2 . Maybe the code can be improved taking this into account, so that the points will not be evenly spaced.
    cout << "previous_path_s.size(): " << previous_path_s.size() << endl;
    
    double aux_car_s = car_s;
    if (prev_size> 0)
        aux_car_s = previous_path_s[prev_size-1];
    

    for(int i = 1; i <= 50 - prev_size; i++) {
        double s_spline = aux_car_s + (i * .02 * ref_vel/2.24);
        double d_spline = spl(s_spline);

        if (i >= 47)
            cout << "next_s_vals[" << i << "]: " << next_s_vals[i] << " ";

        next_s_vals.push_back(s_spline);
        next_d_vals.push_back(d_spline);

        previous_path_s.push_back(s_spline);
        previous_path_d.push_back(d_spline);

    }

    return {next_s_vals, next_d_vals};

}
*/



