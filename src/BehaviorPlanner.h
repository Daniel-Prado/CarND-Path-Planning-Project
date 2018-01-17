#ifndef DP_BEHAVIOR_PLANNER_H
#define DP_BEHAVIOR_PLANNER_H

#include <iostream>
#include <vector>
#include "Vehicle.h"
#include "TrajectoryGenerator.h"
#include "RoadMap.h"

const int REACTION_STEPS = 10;
const int TRAJ_N_STEPS = 50;
const double LANE_WIDTH = 4.0;

const double max_speed = 21.5;
const double max_acc = 10.0;

const auto lane_d = [](size_t lane) {
  return lane * LANE_WIDTH + LANE_WIDTH / 2;
};

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class LaneStates {
	KL, PLCL, PLCR, LCL, LCR
};


using namespace std;

class BehaviorPlanner {

private:

  	TrajectoryGenerator& _traj_gen;
  	RoadMap& _road;

	size_t target_lane;
	LaneStates state;

  	vector<Vehicle> 	generate_prediction(Vehicle& vehicle, size_t horizon_steps);
	vector<LaneStates>	successor_states();
	vector<Vehicle> 	generate_trajectory(LaneStates state, const Vehicle& ego, const vector<vector<Vehicle>>& predictions);
	vector<Vehicle> 	keep_lane_trajectory(const Vehicle& ego, const vector<vector<Vehicle>>& predictions);
	vector<Vehicle> 	find_vehicle_ahead_in_lane(int lane, double s, const vector<vector<Vehicle>> &predictions);
	
	Vehicle 			get_desired_traj_end_position(int desired_lane, Vehicle car_at_start, const vector<vector<Vehicle>> &predictions);
	vector<Vehicle>		get_desired_trajectory(Vehicle car_at_start, Vehicle car_at_goal, int reaction_steps);
	vector<Vehicle> 	follow_old_trajectory(Vehicle car_at_start, size_t n_reaction_steps);

public:
	vector<vector<Vehicle>> make_predictions(const vector<Vehicle> &vehicles, size_t horizon_steps);
	vector<Vehicle> 		make_plan(const Vehicle& ego, const vector<vector<Vehicle>>& predictions);	

	BehaviorPlanner(RoadMap &road, TrajectoryGenerator &traj_generator, const size_t init_lane) : 
						_road(road), _traj_gen(traj_generator), target_lane(init_lane) {state = LaneStates::KL;};
};

#endif
