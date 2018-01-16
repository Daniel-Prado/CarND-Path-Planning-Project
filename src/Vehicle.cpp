#include "Vehicle.h"

Vehicle::Vehicle(const int identifier){
    this->id = identifier;
    this->s = 0;
    this->s_dot = 0;
    this->s_ddot = 0;
    this->d = 0;
    this->d_dot = 0;
    this->d_ddot = 0;
}


void Vehicle::update_pos(double s, double d) {
    this->s = s;
    this->d = d;
}

void Vehicle::update_vel(double s_dot, double d_dot) {
    this->s_dot = s_dot;
    this->d_dot = d_dot;
}

Vehicle Vehicle::position_after_n_seconds(double incr_time) {
  Vehicle future_vehicle = *this; // Copy asignment, state is a new object, copy of this.
  
  //  IN THIS FIRST IMPLEMENTATION WE ASSUME CONSTANT VELOCITY OF OTHER CARS
  //  AND NO LANE CHANGES. (Constant s speed, constant d position)
  future_vehicle.s = s + incr_time * s_dot;
  
  return future_vehicle;
}

int Vehicle::get_lane() const {
    int lane = Vehicle::get_lane(this->d);
    return lane;
}

int Vehicle::get_lane(const double d) {
    
    int lane;

    if (d > 0.0 && d <= 4.0) {
        lane = 0;
    } else if (d > 4.0 && d <= 8.0) {
        lane = 1;
    } else if (d > 8.0 && d <= 12.0) {
        lane = 2;
    }

    return lane;
}


void Vehicle::move_along_trajectory(const vector<double>& traj_s, const vector<double>& traj_d, size_t steps) {
  
  long index = min(steps, traj_s.size()) - 1;
  double dt = 0.02;

  if(index > 0) {

    double s_dot = (traj_s[index] - traj_s[index - 1] )/ dt;
    double d_dot = (traj_d[index] - traj_d[index - 1]) / dt;
    this->s_ddot = (s_dot - this->s_dot) / ((index + 1) * dt);
    this->d_ddot = (d_dot - this->d_dot) / ((index + 1) * dt);
    
    update_pos(traj_s[index], traj_d[index]);
    update_vel(s_dot, d_dot);
  
  } else if(index == 0) {
    double s_dot = (traj_s[index] - this->s) / dt;
    double d_dot = (traj_d[index] - this->d) / dt;
    this->s_ddot = (s_dot - this->s_dot) / dt;
    this->d_ddot = (d_dot - this->d_dot) / dt;
    
    update_pos(traj_s[index], traj_d[index]);
    update_vel(s_dot, d_dot);
  
    }
}
