#ifndef DP_BEHAVIOR_PLANNER_H
#define DP_BEHAVIOR_PLANNER_H

#include <iostream>
#include <vector>
#include "Vehicle.h"
#include "TrajectoryGenerator.h"
#include "RoadMap.h"

const int REACTION_STEPS = 15;
const int TRAJ_N_STEPS = 75;
const double LANE_WIDTH = 4.0;

const double max_speed = 20.0;
const double max_acc = 8.0;

const double s_collision_range = 12.0;
const double d_collision_range = 3;
const double safety_distance = 20.0; // for now, we consider a fix safety distance

const auto lane_d = [](size_t lane) {
  auto d = lane * LANE_WIDTH + LANE_WIDTH / 2;
  if (lane == 2) {
  	d = d - 0.2; // Correction for glitch in the simulator (issue in cartessian-frenet conversion in some curves, in the right lane)
  }
  return d;
};

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class LaneStates {
	KL, PLCL, PLCR, LCL, LCR
};

template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
};


using namespace std;

class BehaviorPlanner {

private:

  	TrajectoryGenerator& _traj_gen;
  	RoadMap& _road;

	size_t target_lane;
	LaneStates state;

  	vector<Vehicle> 	generate_prediction(Vehicle& vehicle, size_t horizon_steps);
  	float 				calculate_cost(const Vehicle& car_at_start, const vector<Vehicle>& trajectory, const vector<vector<Vehicle>> &predictions);
	vector<LaneStates>	successor_states(const int ego_lane);
	vector<Vehicle> 	generate_trajectory(bool try_mode, LaneStates state, const Vehicle& ego, const vector<vector<Vehicle>>& predictions);
	vector<Vehicle> 	keep_lane_trajectory(const Vehicle& ego, const vector<vector<Vehicle>>& predictions);
	vector<Vehicle>		lane_change_trajectory(bool try_mode, LaneStates state, const Vehicle& ego, const vector<vector<Vehicle>>& predictions);
	bool				detect_collision_in_trajectory(const vector<Vehicle>& ego_traj, const vector<vector<Vehicle>>& predictions); 
	vector<Vehicle> 	find_vehicle_ahead_in_lane(int lane, double s, const vector<vector<Vehicle>> &predictions);
	
	Vehicle 			get_desired_traj_end_position(int desired_lane, Vehicle car_at_start, const vector<vector<Vehicle>> &predictions);
	vector<Vehicle>		get_desired_trajectory(Vehicle car_at_start, Vehicle car_at_goal, int reaction_steps);
	vector<Vehicle> 	follow_old_trajectory(Vehicle car_at_start, Vehicle car_at_goal, size_t n_reaction_steps);

public:
	vector<vector<Vehicle>> make_predictions(const vector<Vehicle> &vehicles, size_t horizon_steps);
	vector<Vehicle> 		make_plan(const Vehicle& ego, const vector<vector<Vehicle>>& predictions);	

	BehaviorPlanner(RoadMap &road, TrajectoryGenerator &traj_generator, const size_t init_lane) : 
						_road(road), _traj_gen(traj_generator), target_lane(init_lane) {state = LaneStates::KL;};
};

#endif
