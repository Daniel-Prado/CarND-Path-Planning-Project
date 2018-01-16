#ifndef DP_TRAJECTORY_GENERATOR_H
#define DP_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include "spline.h"
#include "RoadMap.h"
#include "Vehicle.h"

using namespace std;

class TrajectoryGenerator {
private:
	vector<double> previous_path_s;
	vector<double> previous_path_d;

	RoadMap& _road;


public:
  	TrajectoryGenerator(RoadMap& road) : _road(road) {};
  	~TrajectoryGenerator();

	void 					update_previous_path(size_t prev_path_length);
	vector<vector<double>> 	get_previous_trajectory();
	size_t 					get_length();
	vector<vector<double>> 	generate_new_trajectory(vector<Vehicle>& prev_trajectory, Vehicle& car_at_start, Vehicle& car_at_goal, const int n_steps);
	vector<vector<double>>	update_trajectory_for_plan(const vector<Vehicle>& plan);
	
	//vector<vector<double>> 	get_new_frenet_trajectory(int prev_size, double ref_vel, int lane, double car_s, double car_d);


};

#endif