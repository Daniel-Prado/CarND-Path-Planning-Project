#ifndef DP_BEHAVIOR_PLANNER_H
#define DP_BEHAVIOR_PLANNER_H

#include <iostream>
//#include <string>
//#include <utility>
#include <vector>
//#include <map>
#include "Vehicle.h"
#include "TrajectoryGenerator.h"

const int REACTION_STEPS = 10;
const int TRAJ_N_STEPS = 50;
const double LANE_WIDTH = 4.0;

const auto lane_d = [](size_t lane) {
  return lane * LANE_WIDTH + LANE_WIDTH / 2;
};


using namespace std;

class BehaviorPlanner {

private:

  	const TrajectoryGenerator& _traj_gen;
  	const RoadMap& _road;

  	vector<Vehicle> 	generate_prediction(Vehicle& vehicle, int horizon_steps);
	vector<LaneStates>	successor_states();
	vector<Vehicle> 	generate_trajectory(LaneStates state, Vehicle& ego, vector<vector<Vehicle>>& predictions);
	vector<Vehicle> 	keep_lane_trajectory(Vehicle& ego, vector<vector<Vehicle>>& predictions);
	vector<Vehicle> 	find_vehicle_ahead_in_lane(int lane, double s, const vector<vector<Vehicle>> &predictions);
	
	Vehicle 			get_desired_traj_end_position(int desired_lane, Vehicle car_at_start, const vector<vector<Vehicle>> &predictions);
	vector<Vehicle>		get_desired_trajectory(Vehicle car_at_start, Vehicle car_at_goal, int reaction_steps);
	vector<Vehicle> 	follow_old_trajectory(Vehicle car_at_start, int n_reaction_steps);
	
	vector<Vehicle> 	generate_prediction(Vehicle &vehicle, int horizon_steps);


public:
	vector<vector<Vehicle>> make_predictions(const vector<Vehicle> &vehicles, int horizon_steps);
	vector<Vehicle> 		make_plan(const Vehicle& ego, const vector<vector<Vehicle>>& predictions);	

	BehaviorPlanner(RoadMap &road, TrajectoryGenerator &traj_generator, const int init_lane);
};

#endif
