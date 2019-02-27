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
#include "spline.h"
#include "helpers.h"
using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  //Car's lane. Stating at middle lane.
  int lane = 1;

  //Reference velocity.
  double ref_vel = 0; // mph
  double speed_diff = .224;
  const double max_vel = 49.5; // mph

  h.onMessage([&max_vel, &speed_diff, &ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
  &map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	// the data format is [id, x, y, vx, vy, s, d].
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            //PREDICTION

            /***
            The prediction component estimates what actions other objects might take in the future. For example, if
            another vehicle were identified, the prediction component would estimate its future trajectory.
            ***/

            /*
                In this example prediction module we use to find out following
                car ahead is too close, car on the left is too close, and car on the right is too close.

                As explained actual prediction module will be implemented using the approach mentioned above, but this
                highway project doesnt need to predict the trajectory of each vehicle as those vehicles trajectory will
                be on the straight lane.
            */

            if(prev_size > 0) {
                car_s = end_path_s;
            }

            bool car_left = false;
            bool car_right = false;
            bool car_ahead = false;
            double check_speed_ahead;
            double cost_l;
            double cost_r;
            double max_cost_l = 0;
            double max_cost_r = 0;
            double diff_car_s;
            double min_car_diffs = 100;

            for(int i=0; i < sensor_fusion.size(); i++) {

                // the d of the i_th detected car
                float d = sensor_fusion[i][6];

                int check_car_lane;

                /*Currently we assume that we have only three lanes and each lane has 4 meter width. In actual scenario,
                number of lanes an d distance between the lanes and total lanes distance can be detected using computer
                vision technologies. We slightly touched in advanced lane findings in term1.
                */
                if(d > 0 && d < 4) {
                    check_car_lane = 0;
                } else if(d > 4 && d < 8) {
                    check_car_lane = 1;
                } else if(d > 8 and d < 12) {
                    check_car_lane = 2;
                }

                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy); // the speed of the car detected
                double check_car_s = sensor_fusion[i][5]; // the s of the car detected

              //This will help to predict where the vehicle will be in future, project s value out in time
                check_car_s += ((double)prev_size*0.02*check_speed);
                if(check_car_lane == lane) {
                    //A vehicle is on the same lane and check the car is in front of the ego car
                    diff_car_s = check_car_s - car_s;
                    car_ahead |= diff_car_s > 0 && diff_car_s < 30;
                    if (car_ahead && diff_car_s < min_car_diffs ){
                        min_car_diffs = diff_car_s;
                        check_speed_ahead = check_speed;
                    }

                } else if((check_car_lane - lane) == -1) {
                    //A vehicle is on the left lane and check that is in 30 meter range
                    diff_car_s = fabs(car_s - check_car_s);
                    car_left |= diff_car_s < 20;
                    cost_l = 1/ (1+ exp(diff_car_s*diff_car_s)); //cost of change to the left lane
                    if (max_cost_l < cost_l){
                        max_cost_l = cost_l; //record the max cost for left lane change
                    }

                } else if((check_car_lane - lane) == 1) {
                    //A vehicle is on the right lane and check that is in 30 meter range
                    diff_car_s = fabs(car_s - check_car_s);
                    car_right |= diff_car_s < 20;
                    cost_r = 1/ (1+ exp(diff_car_s*diff_car_s)); //cost of change to the right lane
                    if (max_cost_r < cost_r){
                        max_cost_r = cost_r; //record the max cost for left lane change
                    }
                }
            }

            //As we said, actual prediction module gives the possible trajectories from the current timeline to the future of each vehicle.
            //In this highway example, we will have only one possible trajectory for each vehicle and that is why we are
            //using simple approach as above.
            //In complex situation we may need to use model, data, or hybrid approach for prediction module

            //BEHAVIOUR
            /***
            The behavioral planning component determines what behavior the vehicle should exhibit at any point in time.
            For example stopping at a traffic light or intersection, changing lanes, accelerating, or making a left turn onto a new street are all maneuvers that may be issued by this component.
            ***/
            if(car_ahead) {
                if(!car_left && lane > 0 ) {
                  if (lane == 2| max_cost_l <= max_cost_r){
                    lane--;
                  }
                } else if(!car_right && lane !=2 ) {
                    lane++;
                }else {
                    if (ref_vel > check_speed_ahead ){
                        ref_vel -= speed_diff; //reduce the car speed by speed_diff if it's faster than the car ahead
                    } else{
                        ref_vel = check_speed_ahead; //follow the front car with the same speed
                    }
                }
            } else if(ref_vel < max_vel ){
                ref_vel += speed_diff; //add the speed when there's no car ahead and the speed is under the limit
            }
            //In actual case, behaviour planner decides the trajectory based on the cost functions.
            //In this highway example, we may no need to worry about cost functions as we are considering only lane change or reduce speed based on the obstacles.

            //TRAJECTORY
            /***
            Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
            ***/
            vector<double> ptsx;
            vector<double> ptsy;

            //Refrence x,y, and yaw states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous states are almost empty, use the car as a starting point
            if ( prev_size < 2 ) {

                //Use two points that makes path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);

            } else {
                //Redefine the reference point to previous point
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // Setting up target points in the future.
            // the car is at its lane center
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // Making coordinates to local car coordinates, so that the angle between the car's direction and the new
            //y coordinate is 0.
            for ( int i = 0; i < ptsx.size(); i++ ) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              //for the car's coordinates
              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw); //car's heading direction
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw); //car's perpendicular direction
            }

            // Create the spline.
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            vector<double> next_x_vals;
          	vector<double> next_y_vals;

            //For the smooth transition, we are adding previous path points
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate distance y position on 30 m ahead.
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            // Fill up the rest of our path planner after filling it with the previous points, here always output  50
            //points.
            for( int i = 1; i < 50 - prev_size; i++ ) {

              double N = target_dist/(0.02*ref_vel/2.24); // N = the number of points, (1m/s = 2.24 MPH)
              double x_point = x_add_on + target_x/N; // x coordinate for point i
              double y_point = s(x_point); // y coordinate for point i

              x_add_on = x_point; // new x coordinate

              double x_ref = x_point; // record the x local coordinate as x_ref
              double y_ref = y_point; // record the y local coordinate as y_ref

              //Rotate back to normal after rotating it earlier
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw); //rotate the coordinate
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw); //rotate the coordinate

              x_point += ref_x; // shift the x coordinate to the global coordinate
              y_point += ref_y; // shift the y coordinate to the global coordinate

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }


          	json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
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
    } else {
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
