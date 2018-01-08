#include "TrajectoryGenerator.h"

void TrajectoryGenerator::TrajectoryGenerator()
{
}


void TrajectoryGenerator::update_previous_path(size_t prev_path_length) {
    
    size_t consumed_path = previous_path_s.size() - prev_path_length;
    
    // The previous_path_x provided by the simulator contains the points not yet visited by the car (prev_path_length)
    // however our 'locally' stored previous_path_s and _d stores the total 50 points,
    // so we need to delete the points in the past.
    if(prev_path_length > 0 && consumed_path > 0) {
        previous_path_s.erase(previous_path_s.begin(), previous_path_s.begin() + consumed_path);
        previous_path_d.erase(previous_path_s.begin(), previous_path_s.begin() + consumed_path);
    // In the extraordinary rare case that the car has visited ALL the points in the previous path,
    // we just get rid of our local copy (it's past, so it's useless now.)
    // In practice this will not happen because the car has only time to visit 2 or 3 points 
    } else if(prev_path_length == 0) {
        previous_path_s.clear();
        previous_path_d.clear();
    }
}

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
    if (prev_size < 2) {
        cout << "WARNING: prev_size < 2" << endl;
        // use two points that nake the path tangent to the car.
        // This will be executed only at the start.
        double prev_car_a = car_s - 1;
        double prev_car_d = car_d;

        pts_s.push_back(car_s-1);
        pts_s.push_back(car_s);

        pts_d.push_back(car_d);
        pts_d.push_back(car_d);
    }
    else {
        // use the previous path end point as a starting reference
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
    vector<double> next_wp30m = { car_s + 30, 2+4*lane };
    vector<double> next_wp60m = { car_s + 60, 2+4*lane };
    vector<double> next_wp90m = { car_s + 90, 2+4*lane };

    pts_s.push_back(next_wp30m[0]);
    pts_s.push_back(next_wp60m[0]);
    pts_s.push_back(next_wp90m[0]);

    pts_d.push_back(next_wp30m[1]);
    pts_d.push_back(next_wp60m[1]);
    pts_d.push_back(next_wp90m[1]);

    // Create a spline
    tk::spline spl;

    spl.set_points(pts_s, pts_d);

    // Now we are going to build the next_s_vals and next_d_vals that we actually will send to the simulator
    // To make the trajectory changes smooth, we are re-using the part of the previous path trajectory that the
    // car hasn't made yet... For example of a 50 bullet points trajectory, the car maybe has only reached the\
    // 3rd one, so 47 bullets are still reusable.
    // NOTE to myself: This may not work if sudden changes in the trajectory may be required (to avoid an accident for example)
    vector<double> next_s_vals;
    vector<double> next_d_vals;

    for (int i = 0; i < previous_path_s.size(); i++) {
        next_s_vals.push_back(previous_path_s[i]);
        next_d_vals.push_back(previous_path_d[i]);
    }

    // We calculate how to break up spline points so that we travel at our desired reference velocity.
    // NOTE this assumes constant velocity, even if the ref_vel could be accelerating or decelerating even at
    // 10m/s2 . Maybe the code can be improved taking this into account, so that the points will not be evenly spaced.
    for(int i = 1; i <= 50 - previous_path_s.size(); i++) {

        double s_spline = s_add_on + .02 * ref_vel/2.24;
        double d_spline = spl(s_spline);

        x_add_on = s_spline;

        next_s_vals.push_back(s_spline);
        next_d_vals.push_back(d_spline);

        previous_path_s.push_back(s_spline);
        previous_path_d.push_back(d_spline);

    }

    return {next_s_vals, next_d_vals};

}

