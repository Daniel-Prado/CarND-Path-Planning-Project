#ifndef DP_TRAJECTORY_GENERATOR_H
#define DP_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include "spline.h"
#include "RoadMap.h"

using namespace std;

class TrajectoryGenerator {
  vector<double> previous_path_s;
  vector<double> previous_path_d;

  RoadMap* _road;

public:
  //Constructor
  TrajectoryGenerator(RoadMap& road);

  //Destructor
  ~TrajectoryGenerator();

  void update_previous_path(size_t prev_path_length);

  vector<vector<double>> get_new_frenet_trajectory(int prev_size, double ref_vel, int lane, double car_s, double car_d);


};

#endif