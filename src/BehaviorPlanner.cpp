#include "behavior_planner.h"

using namespace std;

enum class Lane {
  LEFT, CENTER, RIGHT, NONE, UNKOWN
};

enum class LaneStates {
	KL, PLCL, PLCR, LCL, LCR
};

typedef int Maneuver;


struct Maneuver
{
	vector<int> wp_lanes;
	vector<double> speeds;
	vector<double> wp_s;
}


BehaviorPlanner::BehaviorPlanner(Vehicle& ego, vector <Vehicle>& vehicles) {

	this->ego = ego;
	this->vehicles = vehicles;
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

vector<Vehicle> BehaviorPlanner::make_predictions(int horizon = 1) {

	vector<Vehicle> pred_vehicles;
	for (auto &pred : this->vehicles) {
		pred.get_prediction(horizon);
		pred_vehicles.push_back(vehicle);
	}

	return pred_vehicles;
}

Maneuver BehaviorPlanner::get_maneuver(LaneStates::state, vector<Vehicle> predictions) {
    
    Maneuver maneuver;
	if (state == LaneStates::KL) {
        maneuver = keep_lane_maneuver(predictions);
    } else if (state == LaneStates::LCL || state == LaneStates::LCR) {
        maneuver = lane_change_maneuver(state, predictions);
    } else if (state == LaneStates::PLCL || state == LaneStates::PLCR) {
        maneuver = prep_lane_change_maneuver(state, predictions);
    }
    return maneuver;


}

vector<Vehicle> BehaviorPlanner::choose_next_state(map<int, vector<Vehicle>> predictions) {

	vector<float> costs;
	vector<maneuver> 

    vector<LaneStates> states = successor_states();

    for (auto &st_it : states) {
    	int maneuver = calculate_maneuver(st_it, predictions)
    	cost = calculate_cost(predictions, maneuver);
    	costs.push_back(cost);
    	final_maneuver.push_back(maneuver);
    }
    
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_maneuver[best_idx];
}




float BehaviorPlanner::calculate_cost(vector<Vehicle> predictions, Maneuver maneuver) {




}




