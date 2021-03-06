#include "BehaviorPlanner.h"


//BehaviorPlanner::BehaviorPlanner(RoadMap &road, TrajectoryGenerator &traj_generator, const int init_lane) {
//    this->_road = &road;
//    this->_traj_gen = &traj_generator;
//    this->init_lane = init_lane;
//}

//BehaviorPlanner::BehaviorPlanner(Vehicle& ego, vector <Vehicle>& vehicles) {
//
//	this->ego = ego;
//	this->vehicles = vehicles;
//}

vector<vector<Vehicle>> BehaviorPlanner::make_predictions(const vector<Vehicle> &vehicles, size_t horizon_steps) {
  
  vector<vector<Vehicle>> predictions;
  
  for(Vehicle vehicle : vehicles) {
    predictions.push_back(generate_prediction(vehicle, horizon_steps));
  }
  
  return predictions;
}

vector<Vehicle> BehaviorPlanner::generate_prediction(Vehicle& vehicle, size_t horizon_steps) {
  
  vector<Vehicle> predictions;
  
  for(int i = 0; i <= horizon_steps; i++) {
    predictions.push_back(vehicle.position_after_n_seconds(i * 0.02));
  }
  return predictions;
}


vector<Vehicle> BehaviorPlanner::make_plan(const Vehicle& ego, const vector<vector<Vehicle>>& predictions) {

    // If we haven't completed a trajectory in course, better to complete it before changing it.
    if(ego.get_lane() != target_lane || fabs(ego.d - lane_d(ego.get_lane())) > LANE_WIDTH / 4) {
        // Get trajectory with the current state
        //DEBUG cout << "CHANGING LANE, target_lane:" << target_lane << endl;
        Vehicle car_at_goal = ego;
        vector<Vehicle> trajectory = this->generate_trajectory(false, state, ego, car_at_goal, predictions);
        return trajectory;
    }
    cout << "AT POSITION " << ego <<"...." << endl;

    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    vector<LaneStates> states = successor_states(ego.get_lane());

    float best_cost = std::numeric_limits<float>::max();
    vector<Vehicle> best_trajectory = {};
    LaneStates best_state;
    size_t best_target = ego.get_lane();

    for (auto &st_it : states) {
        Vehicle car_at_goal = ego; //we will update car_at_goal passing by reference to generate_trajectory.
        vector<Vehicle> trajectory = this->generate_trajectory(true, st_it, ego, car_at_goal, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(ego, car_at_goal, trajectory, predictions);
            //cout << " ---Tried state " << st_it << ", target_lane:" << target_lane << "  cost: " << cost << endl;
            if (cost < (best_cost-1.0)) {
                best_trajectory = trajectory;
                
                if (st_it == LaneStates::KL)
                    best_target = ego.get_lane();
                else if (st_it == LaneStates::LCL)
                    best_target = ego.get_lane() - 1;
                else if (st_it == LaneStates:: LCR)
                    best_target = ego.get_lane() + 1;
                
                best_cost = cost-1.0;
                best_state = st_it;
            }
            //costs.push_back(cost);
            //final_trajectories.push_back(trajectory);
        }
        else
            cout << "generate_trajectory for state " << st_it << " returned {}" << endl;
    }

    cout << " -- decided ST " << best_state << "| target_lane: " << best_trajectory.back().get_lane() << endl;

    this-> state = best_state;
    this-> target_lane = best_target;//best_trajectory.back().get_lane();
    return best_trajectory;

    /*
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    cout << endl << "Current State: " << state;
    this->state = states[best_idx];
    cout << ", next state: " << state << endl;
    this->target_lane = final_trajectories[best_idx].back().get_lane();
    return final_trajectories[best_idx];
    */
}

float BehaviorPlanner::calculate_cost(const Vehicle& car_at_start, const Vehicle& car_at_goal, const vector<Vehicle>& trajectory, const vector<vector<Vehicle>> &predictions) {

    float cost_collision = 0;

    if(car_at_start.get_lane() != car_at_goal.get_lane()) {
        if (detect_collision_in_trajectory(trajectory, predictions))
        {
            cout << " --COSTS/Target_lane: "  << car_at_goal.get_lane() << " COLLISION" << endl;
            cost_collision = 100000;
            return cost_collision;
        }
    }
    float cost_speed = 500 * max ((max_speed - car_at_goal.s_dot) / max_speed, 0.0 );

    float cost_busy_lane = 0;
    vector<Vehicle> car_ahead_predictions = find_vehicle_ahead_in_lane(car_at_goal.get_lane(), car_at_start.s, predictions);
    if (!car_ahead_predictions.empty()) {
        float dist = static_cast<float>(RoadMap::safe_diff(car_ahead_predictions.back().s, car_at_start.s)); // the farther future pos it will be compared with our initial pos, the better.
        
        //Example costs for this function:
        //   dist = 0 :  cost = 500
        //   dist = 10:  cost = 377.5
        //   dist = 20:  cost = 268.9
        //   dist = 50:  cost = 75.8
        //   dist = 100: cost = 6.7
        float max_progress = max_speed * TRAJ_N_STEPS * 0.02;

        if (dist < 2 * max_progress) // ignore traffic farther than this distance
            cost_busy_lane = 1000.0 / (1 + exp(dist/20));
    }else
        cost_busy_lane = 0;

    //marginal cost for changing lane.
    float cost_not_central_lane = 0;
    if (car_at_goal.get_lane()!= 1)
        cost_not_central_lane = 10;

    cout << " --COSTS/Target_lane: "  << car_at_goal.get_lane() << " | Total: "<< cost_speed+cost_busy_lane+cost_not_central_lane << " | cost_speed: " 
         << cost_speed << " | cost_busy: " << cost_busy_lane << " | cost_not_central: " << cost_not_central_lane <<endl;

    return cost_speed + cost_busy_lane + cost_not_central_lane;
}

vector<LaneStates> BehaviorPlanner::successor_states(const int ego_lane) {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<LaneStates> states;

    /* FULL VERSION
    //Keep current lane is always an option
    states.push_back(LaneStates::KL);
    if(state == LaneStates::KL) {
        if (this->ego.current_lane > 0) 
            states.push_back(LaneStates::PLCL);
        if (this->ego.current_lane < 2)
            states.push_back(LaneStates::PLCR);
    } else if (state == LaneStates::PLCL) {
        if (this->ego.current_lane > 0) {
            states.push_back(LaneStates::PLCL);
            states.push_back(LaneStates::LCL);
        }
    } else if (state == LaneStates::PLCR) {
        if (this->ego.current_lane < 2) {
            states.push_back(LaneStates::PLCR);
            states.push_back(LaneStates::LCR);
        }
    } else if (state == LaneStates::LCL)  {
            states.push_back(LaneStates::LCL);
    
    } else if (state == LaneStates::LCR)  {
            states.push_back(LaneStates::LCR);
    }
    */
    // SIMPLIFIED VERSION: (No PLCL, No PLCR)
    //Keep current lane is always an option
    states.push_back(LaneStates::KL);
    if(state == LaneStates::KL) {
        if (ego_lane > 0) 
            states.push_back(LaneStates::LCL);
        if (ego_lane < 2)
            states.push_back(LaneStates::LCR);
    } else if (state == LaneStates::LCL)  {
        if (ego_lane > target_lane)
            states.push_back(LaneStates::LCL);
    
    } else if (state == LaneStates::LCR)  {
        if(ego_lane < target_lane)
            states.push_back(LaneStates::LCR);
    }
    return states;
}

vector<Vehicle> BehaviorPlanner::generate_trajectory(bool try_mode, LaneStates state, const Vehicle& ego, Vehicle& car_at_goal, const vector<vector<Vehicle>>& predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    
    if (state == LaneStates::KL) {
        trajectory = keep_lane_trajectory(ego, car_at_goal, predictions);
    } else if (state == LaneStates::LCL || state == LaneStates::LCR) {
        trajectory = lane_change_trajectory(try_mode, state, ego, car_at_goal, predictions);

    } else if (state == LaneStates::PLCL || state == LaneStates::PLCR) {
        //trajectory = prep_lane_change_trajectory(state, ego, predictions);
        // PROVISIONAL
        cout << "ERROR....NOT IMPLEMENTED" << endl;
        trajectory = {};
    }
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::keep_lane_trajectory(const Vehicle& ego, Vehicle& car_at_goal, const vector<vector<Vehicle>>& predictions) {
    /*
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
    // End of classroom quiz code.
    */

    /* DEBUG
    cout << "target_lane: " << target_lane << "ego:" << endl;
    cout << ego << endl;
    cout << "Desired end position:" << endl;
    */
    Vehicle desired_end_position = this->get_desired_traj_end_position(target_lane, ego, predictions);
    // DEBUG cout << desired_end_position << endl;
    vector<Vehicle> best_trajectory = this->get_desired_trajectory(ego, desired_end_position, REACTION_STEPS);

    car_at_goal = desired_end_position;
    return best_trajectory;
}

vector<Vehicle> BehaviorPlanner::lane_change_trajectory(bool try_mode, LaneStates state, const Vehicle& ego, Vehicle& car_at_goal, const vector<vector<Vehicle>>& predictions) {
    /*
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
    // End of classroom quiz code.
    */
    int line_to_try;

    if (try_mode) {
        if (state == LaneStates::LCL) {
            line_to_try = target_lane - 1;
        } else if (state == LaneStates::LCR) {
            line_to_try = target_lane + 1;
        }
        cout << " --ego.get_lane: " << ego.get_lane() << ", eval ST " << state << ", target_lane: " << line_to_try << endl;
    }
    else {
        line_to_try = target_lane;
        cout << " --ego.get_lane: " << ego.get_lane() << ", executing ST " << state << ", target_lane: " << line_to_try << endl;
    }

    vector<Vehicle> best_trajectory = {};
    // if end position in trajectory is not feasible, will return ego.
    Vehicle desired_end_position = this->get_desired_traj_end_position(line_to_try, ego, predictions);
    car_at_goal = desired_end_position;

    // DEBUG if(desired_end_position.get_lane() != line_to_try)
    //          cout << endl << "WARN ! END_POSITION.GET_LANE and LINE_TO_TRY don't MATCH !!!" << endl << endl;
    if (RoadMap::safe_diff(desired_end_position.s,ego.s) > 0)
    {
        best_trajectory = this->get_desired_trajectory(ego, desired_end_position, REACTION_STEPS);
        //DEBUG if(best_trajectory.back().get_lane() != desired_end_position.get_lane())
        //          cout << endl << "WARN ! END_POSITION.GET_LANE and TRAJ.BACK.GET_LANE don't MATCH !!!" << endl << endl;
        // else, we return empty trajectory
    }
    return best_trajectory;
}

bool BehaviorPlanner::detect_collision_in_trajectory(const vector<Vehicle>& ego_traj, const vector<vector<Vehicle>>& predictions) {

    //size_t length = min<unsigned long>(20, ego_traj.size());
    int length = ego_traj.size();
    bool collision = false;
    int i = 0;
    for (const vector<Vehicle> &prediction : predictions) {
        i = 0;
        while (!collision && i < length) {
            // note that we use prediction[0].d instead of prediction[i].d because the prediction in d coordinate is not reliable.
            if(fabs(RoadMap::safe_diff(ego_traj[i].s, prediction[i].s)) < s_collision_range && fabs(ego_traj[i].d - prediction[0].d) < d_collision_range) {
                //cout << "DETECTED COLLISION with lane " << prediction[0].get_lane() << endl;
                //cout << " -- between car "<< prediction[0] << "and ego " << ego_traj[0] << endl;
                collision = true;
            }
            i++;
        }
        // The trajectory check above sometimes can fail if the speed prediction of the other cars, specially in the d coordinate, is wrong.
        // So, we will treat any car at the sides like a collision.
        /*if (!collision) {
            if(fabs(RoadMap::safe_diff(ego_traj[0].s, prediction[0].s)) < s_collision_range && fabs(ego_traj[0].d - prediction[0].d) < 1.2*LANE_WIDTH) {
                if (ego_traj.back().get_lane() == prediction[0].get_lane())
                    cout << "DETECTED COLLISION TYPE 2" << endl;
                    collision = true;
            }
        }
        */
        if (collision) break;
    }

    return collision;
}

vector<Vehicle> BehaviorPlanner::find_vehicle_ahead_in_lane(int lane, double s, const vector<vector<Vehicle>> &predictions) {

    double car_ahead_dist = circuit_max_s;

    vector<Vehicle> car_ahead_predictions;
    for (const vector<Vehicle> &prediction : predictions) {
        // note that prediction[] is the vector of predicted positions for a given car,
        // hence, the [0] corresponds to the current position.
        // NOTE: As a future improvement, we could check if a car is predicted to be ahead at any given predicted position.
        if(lane == prediction[0].get_lane()) {
            double dist = RoadMap::safe_diff(prediction[0].s , s);
            if(dist < car_ahead_dist && dist >= 0) {
                car_ahead_dist = dist;
                car_ahead_predictions = prediction;
            }
        }
    }

    return car_ahead_predictions;
}

Vehicle BehaviorPlanner::get_desired_traj_end_position(int desired_lane, Vehicle car_at_start, const vector<vector<Vehicle>> &predictions) {
    Vehicle car_at_goal = car_at_start;

    double time_horizon = TRAJ_N_STEPS * 0.02;
    double target_s_dot = min(max_speed, car_at_start.s_dot + max_acc * time_horizon);
    double acc = (target_s_dot - car_at_start.s_dot) / time_horizon;
    double s_incr = car_at_start.s_dot * time_horizon + 0.5 * acc * pow(time_horizon, 2);
    if (s_incr < 0)
        cout << endl << "ERROR s_incr < 0" << endl;
    // Find car ahead
    vector<Vehicle> car_ahead_predictions = find_vehicle_ahead_in_lane(desired_lane, car_at_start.s, predictions);
    
    double safety_distance = safety_distance_fix  + car_at_start.s_dot * REACTION_TIME + 0.5 * 2 * max_acc * pow(REACTION_TIME, 2);

    if (!car_ahead_predictions.empty()) {
        double target_distance = RoadMap::safe_diff(car_ahead_predictions.back().s, car_at_start.s) - safety_distance;
        if (target_distance <= 0) {
            // we cannot change safely because of the safety distance, so it's no sense to consider this trajectory
            cout << "target_distance negative" << endl;
            return car_at_start;
        }
        else if (target_distance < s_incr) {
            target_s_dot = min(max_speed, car_ahead_predictions.back().s_dot);
            s_incr = target_distance;
        }
    }
    

    double target_d = lane_d(desired_lane);
    /*if(desired_lane != car_at_start.get_lane()) {
        target_d = lane_d(desired_lane);
    }
    else {
        target_d = lane_d(car_at_start.get_lane());
    }
    */

    car_at_goal.s = RoadMap::safe_s(car_at_start.s + s_incr);
    car_at_goal.s_dot = target_s_dot;
    car_at_goal.s_ddot = 0;
    car_at_goal.d = target_d;
    car_at_goal.d_dot = 0;
    car_at_goal.d_ddot = 0;
    // we consider a negative acceleration in the d coordinate because that's logical when "braking" in the d coordinate
    // in order to stay in that lane.
    car_at_goal.d_ddot = 0.5 * max_acc * (car_at_start.d - target_d) / LANE_WIDTH;
    
    return car_at_goal;
}

vector<Vehicle> BehaviorPlanner::get_desired_trajectory(Vehicle car_at_start, Vehicle car_at_goal, int reaction_steps) {

    vector<Vehicle> trajectory = this->follow_old_trajectory(car_at_start, reaction_steps);

    int n_future_steps = TRAJ_N_STEPS - trajectory.size();

    Vehicle car_at_start_ii = car_at_start;
    if(!trajectory.empty())
        car_at_start_ii = trajectory.back();


    vector<vector<double>> traj_second = this->_traj_gen.generate_new_trajectory(trajectory, car_at_start_ii, car_at_goal, n_future_steps);
    
    // DEBUG cout << "generate_new_trajectory first s[0]: " << traj_second[0][0] << ", last s[" << n_future_steps << "]: " << traj_second[0][n_future_steps-1] << endl;
    for(int i = 1; i <= n_future_steps; i++) {
        Vehicle veh = car_at_start;
        veh.move_along_trajectory(traj_second[0], traj_second[1], i);
        trajectory.push_back(veh);
    }
    /* DEBUG
    cout << "final path trajectory size: " << trajectory.size() << endl;
    cout << " -- car position at end of trajectory: " << endl;
    cout << trajectory[49] << endl; 
    */
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::follow_old_trajectory(Vehicle car_at_start, size_t n_reaction_steps) {

  vector<vector<double>> old_traj = this->_traj_gen.get_previous_trajectory();
  size_t steps = min(this->_traj_gen.get_length(), n_reaction_steps);

  vector<Vehicle> trajectory = {};
  // The car continues the old trajectory until the new trajectory starts after n_reaction_steps.
  for(int i = 1; i <= steps; i++) {
    Vehicle veh = car_at_start;
    // we need to be careful not no approach too much to the s goal using the OLD trajectory
    // for that we want to use the new one.
    //if (old_traj[i][0] < car_at_goal.s-3.0) 
        veh.move_along_trajectory(old_traj[0], old_traj[1], i);
    //else
    //    break;
    trajectory.push_back(veh);
  }
  return trajectory;
}

//float BehaviorPlanner::calculate_cost(vector<Vehicle> predictions, vector<Vehicle>& traj) {
//    return 0.0;
//}




