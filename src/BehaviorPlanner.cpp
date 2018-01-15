#include "behavior_planner.h"

using namespace std;

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class LaneStates {
	KL, PLCL, PLCR, LCL, LCR
};

//typedef int Maneuver;


struct Maneuver
{
	vector<int> wp_lanes;
	vector<double> speeds;
	vector<double> wp_s;
}


BehaviorPlanner::BehaviorPlanner(RoadMap &road, TrajectoryGenerator &traj_generator, const int init_lane) {
    this->_road = &road;
    this->_traj_gen = &traj_generator;
    this->init_lane = init_lane;

    this->state = LaneStates::KL;
}

//BehaviorPlanner::BehaviorPlanner(Vehicle& ego, vector <Vehicle>& vehicles) {
//
//	this->ego = ego;
//	this->vehicles = vehicles;
//}

vector<vector<Vehicle>> BehaviorPlanner::make_predictions(const vector<Vehicle> &vehicles, int horizon_steps) {
  
  vector<vector<Vehicle>> predictions;
  
  for(Vehicle vehicle : vehicles) {
    predictions.push_back(generate_prediction(vehicle, horizon));
  }
  
  return predictions;
}

vector<Vehicle> BehaviorPlanner::generate_prediction(Vehicle& vehicle, int horizon_steps) {
  
  vector<Vehicle> predictions;
  
  for(int i = 0; i <= horizon_steps; i++) {
    predictions.push_back(vehicle.position_after_n_seconds(i * 0.02));
  }
  return predictions;
}


vector<Vehicle> BehaviorPlanner::make_plan(Vehicle& ego, vector<vector<Vehicle>>& predictions) {

    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    vector<LaneStates> states = successor_states();

    for (auto &st_it : states) {
        vector<Vehicle> trajectory = generate_trajectory(st_it, ego, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }
    
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];

}


vector<LaneStates> BehaviorPlanner::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<LaneStates> states;

    string state = this->state;
    
    //Keep current lane is always an option
    states.push_back(LaneStates::KL);
    if(state == LaneStates::KL) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state == LaneStates::PLCL) {
        if (this->ego.current_lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state == LaneStates::PLCR) {
        if (this->ego.current_lane != 2) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    } else if (state == LaneStates::LCL)  {
            states.push_back("LCL");
    
    } else if (state == LaneStates::LCR)  {
            states.push_back("LCR");
    }

    return states;
}

vector<Vehicle> BehaviorPlanner::generate_trajectory(LaneStates state, Vehicle& ego, vector<vector<Vehicle>>& predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    
    if (state == LaneStates::KL) {
        trajectory = keep_lane_trajectory(ego, predictions);
    } else if (state == LaneStates::LCL || state == LaneStates::LCR) {
        trajectory = lane_change_trajectory(state, ego, predictions);
    } else if (state == LaneStates::PLCL || state == LaneStates::PLCR) {
        trajectory = prep_lane_change_trajectory(state, ego, predictions);
    }
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::keep_lane_trajectory(Vehicle& ego, vector<vector<Vehicle>>& predictions) {
    /*
    Generate a keep lane trajectory.
    
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
    // End of classroom quiz code.
    */
    int target_lane = Vehicle::get_lane(ego.d);

    Vehicle desired_end_position = this->get_desired_traj_end_position(target_lane, ego, predictions);
    vector<Vehicle> best_trajectory = this->get_desired_trajectory(ego, desired_end_position, REACTION_STEPS);

}

vector<Vehicle> BehaviorPlanner::find_vehicle_ahead_in_lane(int lane, double s, const vector<vector<Car>> &predictions) {

    double car_ahead_dist = 6945.554; // max s of the circuit.

    vector<Vehicle> car_ahead_predictions;
    for (const vector<Vehicle> &prediction : predictions) {
        // note that prediction[] is the vector of predicted positions for a given car,
        // hence, the [0] corresponds to the current position.
        // NOTE: As a future improvement, we could check if a car is predicted to be ahead at any given predicted position.
        if(lane == prediction[0].getLane()) {
            double dist = prediction[0].s - s;
            if(dist < car_ahead_dist && dist >= 0) {
                car_ahead_dist = dist;
                car_ahead = prediction;
            }
        }
    }

    return car_ahead_predictions;
}

Vehicle BehaviorPlanner::get_desired_traj_end_position(int desired_lane, Vehicle car_at_start, const vector<vector<Vehicle>> &predictions) {
    Vehicle car_at_goal = car_at_start;

    // Find car ahead
    vector<Vehicle> car_ahead_predictions = find_vehicle_ahead_in_lane(goal_lane, car_at_start.s, predictions);

    double time_horizon = TRAJ_N_STEPS * 0.02;
    double goal_s_dot = min(max_speed, car_at_start.s_dot + max_acc * time_horizon);
    double acc = (target_s_dot - start.s_dot) / time_horizon;
    double s_incr = car_at_start.s_dot * time_horizon + 0.5 * acc * pow(time_horizon, 2);
    
    if (!car_ahead_predictions.empty()) {
        double safety_distance = 15.0; // for now, we consider a fix safety distance
        double goal_distance = car_ahead_predictions.back().s - safety_distance - car_at_start.s;
        if (goal_distance < s_incr && goal_distance > 0) {
            goal_s_dot = car_ahead_predictions.back().s_dot;
            s_incr = goal_distance;
        }
    }
    
    if(desired_lane != car_at_start.get_lane()) {
        target_d = lane_d(desired_lane);
    }
    else {
        target_d = lane_d(car_at_start.get_lane());
    }

    car_at_goal.s = car_at_start.s + s_incr;
    car_at_goal.s_dot = goal_s_dot;
    car_at_goal.s_ddot = 0;
    car_at_goal.d = target_d;
    car_at_goal.d_dot = 0;
    car_at_goal.d_ddot = 0;
    //car_at_goal.d_ddot = 0.8 * max_acc * (start.d - target_d) / LANE_WIDTH;
    
    return car_at_goal;
}

vector<Vehicle> BehaviorPlanner::get_desired_trajectory(Vehicle car_at_start, Vehicle car_at_goal, int reaction_steps) {

    vector<Vehicle> trajectory = this->followTrajectory(car_at_start, reaction_steps);
    int n_future_steps = TRAJ_N_STEPS - trajectory.size();

    if(!trajectory.empty()) {
        car_at_reaction = trajectory.back();
    }
    
    Vehicle veh;
    vector<vector<double>> traj_second = this->_traj_gen.generate(car_at_start, car_at_goal, n_future_steps);
    
    for(int i = 1; i <= n_future_steps; i++) {
        veh = car_at_start;
        veh.follow_trajectory(traj_second[0], traj_second[1], i);
        trajectory.emplace_back(veh);
    }

    return trajectory;
}

vector<Car> BehaviorPlanner::followTrajectory(Car start, size_t steps) const {
  // Limit the delay to the length of old_path_s
  vector<vector<double>> head = traj.getTrajectory();
  size_t delay = min(traj.getTrajectoryLength(), steps);

  vector<Car> path;
  Car curr;
  // Move the car forward
  for(size_t i = 1; i <= delay; i++) {
    curr = start;
    curr.followTrajectory(head[0], head[1], i);
    path.emplace_back(curr);
  }

  return path;
}

vector<vector<Vehicle>> BehaviorPlanner::make_predictions(const vector<Vehicle> &vehicles, int horizon_steps) {
  
    vector<vector<Vehicle>> predictions;
  
    for(Vehicle vehicle : vehicles) {
        predictions.push_back(generate_prediction(vehicle, horizon));
  }
  
  return predictions;
}

vector<Vehicle> BehaviorPlanner::generate_prediction(Vehicle &vehicle, int horizon_steps) {
  
    vector<Vehicle> predictions;
  
    for(int i = 0; i <= horizon_steps; i++) {
        predictions.push_back(vehicle.position_after_n_seconds(i * 0.02));
    }
    return predictions;
}



float BehaviorPlanner::calculate_cost(vector<Vehicle> predictions, Maneuver maneuver) {




}




