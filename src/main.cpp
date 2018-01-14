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


#define FIX_SAFETY_DISTANCE 20
#define HORIZON 1

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


	h.onMessage([&road, &traj_generator,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

                    // position of my car in future.
                    double my_car_s;
                    if (prev_size > 0) {
                        my_car_s = end_path_s;
                        cout << "end_path_s:" << end_path_s << endl;
                    }
                    else
                        my_car_s = car_s;
                    	cout << "initial car_s: " << car_s << endl;

                    bool too_close = false;

                    Vehicle ego;
                    vector<Vehicle> vehicles;

                    ego.set_status_ego(car_s, car_d, car_x, car_y, car_yaw, car_speed/2.24);
                    
                    for(auto &sf_it : sensor_fusion) {
                        int id    = sensor_fusion[i][0];
                        double x  = sensor_fusion[i][1];
                        double y  = sensor_fusion[i][2];
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double s  = sensor_fusion[i][5];
                        double d  = sensor_fusion[i][6];

                        Vehicle other_vehicle(id);

                        other_vehicle.set_status(s, d, sqrt( vx*vx + vy*vy ));

                        vehicles.push_back(other_vehicle);
                    }

                    BehaviorPlanner bp(ego, vehicles);
                    vector<Vehicle> predictions = bp.generate_predictions(HORIZON);



                    //Refresh previous Path to used by the Trajectory Generator and Behavior Planner
                    //The previous_path_x provided by the simulator contains the points not yet visited by the car,
                    //however our 'locally' stored previous_path_s and _d stores the total 50 points, so let's
                    //aling that.
                    traj_generator.update_previous_path(prev_size);
                    cout << "AAA" << endl;

                    // Determine the ref_v to use based on the sorrounding vehicles information incuded in the sensor fusion.
                    for(auto &sf_it : sensor_fusion) {
                        // if car is in my lane
                        float d = sf_it[6];
                        if ( d < (2+4*lane+2) && d > (2+4*lane-2) ) {

                            double vx = sf_it[3];
                            double vy = sf_it[4];
                            double check_speed = sqrt( vx*vx + vy*vy);
                            double check_car_s = sf_it[5];

                            check_car_s += ((double)prev_size * .02*check_speed);
                            double safety_distance = FIX_SAFETY_DISTANCE;

                            //check s values greater than mine and s gap
                            if ((check_car_s > my_car_s) && (check_car_s - my_car_s) < safety_distance) {
                                // do some logic here
                                // reduce velocity or activate flag to change lanes
                                // ref_vel = check_speed * 2.24;
                                too_close = true;
                                if (lane == 0)
                                {
                                    lane = 1;

                                }
                                else if (lane == 1)
                                {
                                    lane = 0;
                                }
                            }
                        }

                    }

                    // Adjust car speed depending on conditions
                    
                    if (too_close){

                        // taking into account that the simulator goes at 50Hz, so that every point is separated 0.02s,
                        // then substracting 0.224 MPH represents an acceleration of about 5 m/s2, which is below the 10m/s2 limit
                        ref_vel -= .224;
                    }
                    else if (ref_vel < 48) {

                        ref_vel += .224;
                    }
                    
  					cout << "BBB" << endl;
                    vector<vector<double>> next_frenet_traj = traj_generator.get_new_frenet_trajectory(prev_size, ref_vel, lane, my_car_s, car_d);
                    vector<double> next_s = next_frenet_traj[0];
                    vector<double> next_d = next_frenet_traj[1];
                    cout << "next_s[0]:" << next_s[0] << "next_d[0]:" << next_d[0] << endl;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    
                    for (int i = 0; i < prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    for(int i = prev_size; i < next_s.size(); i++) {
                    	
                        //vector<double> next_point = getXY(next_s[i], next_d[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_point = road.get_splined_xy(next_s[i], next_d[i]);
                        next_x_vals.push_back(next_point[0]);
                        next_y_vals.push_back(next_point[1]);
                        if (i >= 47)
                        	cout << "next_s[" << i << "]: " << next_s[i] << " ";
                    }
                    cout << endl;

                    json msgJson;

                    cout << "size of next_x_vals sent: " << next_x_vals.size() << endl;

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
