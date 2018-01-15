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

class BehaviourPlanner {
  size_t target_lane;
  
  const TrajectoryGenerator& _traj_gen;
  const RoadMap& _road;


  Car generateGoal(size_t goal_lane, Car start, const vector<vector<Car>> &predictions) const;
  vector<Car> findLeaderInLane(size_t lane, double s, const vector<vector<Car>> &predictions) const;
  vector<Car> generateTrajectory(Car start, Car goal, size_t delay) const;
  vector<Car> followTrajectory(Car start, size_t steps) const;
  double calculateCost(Car start, Car goal, const vector<Car>& trajectory, const vector<vector<Car>> &predictions) const;
public:
  explicit BehaviourPlanner(const TrajectoryGenerator& traj_, size_t initial_lane): traj(traj_), target_lane(initial_lane) {};
  vector<Car> updatePlan(const Car& ego, const vector<vector<Car>>& predictions);
};

#endif
