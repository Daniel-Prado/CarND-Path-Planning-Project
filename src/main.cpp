#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "RoadMap.h"
#include "TrajectoryGenerator.h"
#include "BehaviorPlanner.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline vector<double> xy_to_polar(double x, double y) {
  
  double speed = sqrt(x * x + y * y);
  double theta = atan2(y, x);
  
  if(theta < 0) theta += 2 * M_PI;
  
  return {speed, theta};
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
  	double map_x = maps_x[i];
  	double map_y = maps_y[i];
  	double dist = distance(x,y,map_x,map_y);
  	if(dist < closestLen)
  	{
  		closestLen = dist;
  		closestWaypoint = i;
  	}

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);

	angle = min(2*pi() - angle, angle);

	if(angle > pi()/4)
	{
    	closestWaypoint++;
    	if (closestWaypoint == maps_x.size())
    	{
    		closestWaypoint = 0;
    	}
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

  // calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> previous_path_s;
vector<double> previous_path_y;

int main() {
	uWS::Hub h;

    // Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";

	RoadMap road(map_file_);

    
    // The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

    int lane = 1; // my lane (central)

    double ref_vel = 0.0; // initial velocity.


    TrajectoryGenerator traj_generator(road);
    BehaviorPlanner b_planner(road, traj_generator, lane);
    Vehicle ego(999);


	h.onMessage([&ego, &b_planner, &road, &traj_generator,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event
	//auto sdata = string(data).substr(0, length);
	//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];

                    // Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					int prev_size = previous_path_x.size();
					cout << "initial prev_size: " << prev_size << endl;
                    cout << "sim.s: " << car_s << ",  sim.d:" <<car_d<< ", sim.speed: " << car_speed << endl;

                    
                    /**************************************************************************
                     **   STEP 1 - Update ego vehicle position along the trajectory
                     **************************************************************************/
                    size_t steps_consumed = traj_generator.get_length() - previous_path_x.size();
                    cout << "steps_consumed: " << steps_consumed << endl;
                    if (steps_consumed > 0 ) {
                        // We need to update the current position of the vehicle along the previous trajectory.
                        // Note that we can do that because we know for sure that the position of the car cannot be outside of the 
                        // the trajectory we sent to the simulator (except if the car collides).
                        // The avantage of updating the car's position along the trajectory instead of just taking the fresh data
                        // from the simulator is that this way we can update the internal ego variables (speed and acceleration).
                        vector<vector<double>> trajectory = traj_generator.get_previous_trajectory();
                        ego.move_along_trajectory(trajectory[0], trajectory[1], steps_consumed);
                    } else {
                        // If there is not previous_trajectory, we just update the position based on the 
                        // data from the simulator.
                        ego.update_pos(car_s, car_d);
                        auto frenet_vel = road.get_frenet_vel(car_s, car_d, car_speed/2.24, deg2rad(car_yaw));
                        cout << "frenet_vel: " << frenet_vel[0] << "#" << frenet_vel[1] << endl;
                        ego.update_vel(frenet_vel[0], frenet_vel[1]);
                    }

                    /*************************************************************************************
                    **  STEP 2 - Refresh previous Path to be used by the Trajectory Generator and Behavior Planner
                    **************************************************************************************/
                    // The previous_path_x provided by the simulator contains the points not yet 
                    // visited by the car, however our 'locally' stored previous_path_s and _d stores 
                    // the total N points, so let's aling that.
                    cout << "calling traj_generator.update_previous_path(" << prev_size << ");" << endl;
                    traj_generator.update_previous_path(prev_size);

                    /**************************************************************************
                     **  STEP 3 - Load sensor fusion data
                     **************************************************************************/
                    vector<Vehicle> vehicles;
                    
                    for(auto &sf_it : sensor_fusion) {
                        int id    = sf_it[0];
                        double x  = sf_it[1];
                        double y  = sf_it[2];
                        double vx = sf_it[3];
                        double vy = sf_it[4];
                        double s  = sf_it[5];
                        double d  = sf_it[6];

                        // We consider only cars inside the 3 lanes, not in opposite direction.
                        if (Vehicle::get_lane(d) != -1) {
                            Vehicle other_vehicle(id);
                            other_vehicle.update_pos(s, d);

                            auto polar_vel = xy_to_polar(vx, vy);
                            auto frenet_vel = road.get_frenet_vel(s, d, polar_vel[0], polar_vel[1]);
                            other_vehicle.update_vel(frenet_vel[0], frenet_vel[1]);
                            //cout << other_vehicle << endl;
                            vehicles.push_back(other_vehicle);
                        }

                    }

                    /**************************************************************************
                     **  STEP 4 - Predict vehicles movement and decide Plan based on predictions
                     **************************************************************************/
  
                    auto predictions = b_planner.make_predictions(vehicles, TRAJ_N_STEPS);
                    //cout << "predictions of first car : *********************" << endl;
                    //for(int i=0; i< TRAJ_N_STEPS; i++) {
                    //    cout << predictions[0][i] << endl;
                    //}


                    auto plan = b_planner.make_plan(ego, predictions);
                    cout << "plan made OK" << endl;

                    /**************************************************************************
                     **  STEP 5 - Get an updated trajectory based on the new plan
                     **************************************************************************/                    
                    vector<vector<double>> next_XY_traj = traj_generator.update_trajectory_for_plan(plan);
 
                    vector<double> next_x_vals = next_XY_traj[0];
                    vector<double> next_y_vals = next_XY_traj[1];

                    json msgJson;

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}

			}
			else {
                // Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
		size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		}
		else {
            // i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
