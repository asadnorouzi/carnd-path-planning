#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int my_lane = 1;
  double ref_vel = 0.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &my_lane, &ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // update car's s coordinate based on the previous path (to avoid collisions!)
          int prev_path_size = previous_path_x.size();
          if (prev_path_size > 0)
            car_s = end_path_s;

          // prediction of other cars behavior/location
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;

          for (unsigned int i=0; i<sensor_fusion.size(); i++) {
            // predict other car's lane
            float other_car_d = sensor_fusion[i][6];
            int other_car_lane = -1;

            if (other_car_d >= 0 && other_car_d <= 4)
              other_car_lane = 0;
            else if (other_car_d > 4 && other_car_d <= 8)
              other_car_lane = 1;
            else if (other_car_d > 8 && other_car_d <= 12)
              other_car_lane = 2;

            if (other_car_lane < 0)
              continue;

            //predict other car's speed
            double other_car_vel_x = sensor_fusion[i][3];
            double other_car_vel_y = sensor_fusion[i][4];
            double other_car_vel = sqrt(pow(other_car_vel_x, 2) + pow(other_car_vel_y, 2));

            //predict other car's distance
            double other_car_s = sensor_fusion[i][5];
            other_car_s += ((double)prev_path_size * 0.02 * other_car_vel);

            int gap_buf = 30;
            if (other_car_lane == my_lane)
              car_ahead |= other_car_s > car_s && other_car_s - car_s < gap_buf;
            else if (other_car_lane - my_lane == -1)
              car_left |= car_s - other_car_s < gap_buf && other_car_s - car_s < gap_buf;
            else if (other_car_lane - my_lane == 1)
              car_right |= car_s - gap_buf < other_car_s && other_car_s - gap_buf < car_s;
          }

            //start planning
            double new_vel = 0.0;
            const double MAX_VEL = 49.50;
            const double MAX_ACC = 0.224;

            if (car_ahead) {
              if (!car_left && my_lane != 0)
               //move to left lane
                my_lane--;
              else if (!car_right && my_lane != 2)
                //move to right lane
                my_lane++;
              else
                //do not change lane!
                new_vel -= MAX_ACC;
            }
            else {
              //no car ahead
              if (ref_vel < MAX_VEL)
                new_vel += MAX_ACC;
            }

            //create the trajectory waypoints
            vector<double> waypoints_x;
            vector<double> waypoints_y;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_path_size < 2) {
              //we use car location as reference
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              waypoints_x.push_back(prev_car_x);
              waypoints_x.push_back(car_x);
              waypoints_y.push_back(prev_car_y);
              waypoints_y.push_back(car_y);
            }
            else {
              //we use previous path as reference
              ref_x = previous_path_x[prev_path_size-1];
              ref_y = previous_path_y[prev_path_size-1];

              double prev_ref_x = previous_path_x[prev_path_size-2];
              double prev_ref_y = previous_path_y[prev_path_size-2];

              ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

              waypoints_x.push_back(prev_ref_x);
              waypoints_x.push_back(ref_x);
              waypoints_y.push_back(prev_ref_y);
              waypoints_y.push_back(ref_y);
            }

          int waypoints_gap = 30;

          //calculate waypoints such that they're 30m apart.
          vector<double> next_waypoint_0 = getXY(car_s + waypoints_gap,
                                                 (2 + 4 * my_lane), map_waypoints_s,
                                                 map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_1 = getXY(car_s + waypoints_gap*2,
                                                 (2 + 4 * my_lane), map_waypoints_s,
                                                 map_waypoints_x, map_waypoints_y);
          vector<double> next_waypoint_2 = getXY(car_s + waypoints_gap*3,
                                                 (2 + 4 * my_lane), map_waypoints_s,
                                                 map_waypoints_x, map_waypoints_y);

          waypoints_x.push_back(next_waypoint_0[0]);
          waypoints_x.push_back(next_waypoint_1[0]);
          waypoints_x.push_back(next_waypoint_2[0]);
          waypoints_y.push_back(next_waypoint_0[1]);
          waypoints_y.push_back(next_waypoint_1[1]);
          waypoints_y.push_back(next_waypoint_2[1]);

          for (unsigned int i=0; i<waypoints_x.size(); i++) {
            //shifting the waypoints coordinates to the car's local coordinate
            double shift_x = waypoints_x[i] - ref_x;
            double shift_y = waypoints_y[i] - ref_y;

            waypoints_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            waypoints_y[i] = shift_x * sin(0 - ref_yaw) - shift_y * cos(0 - ref_yaw);
          }

          //create the spline
          tk::spline spl;
          spl.set_points(waypoints_x, waypoints_y);

          for (int i=0; i<prev_path_size; i++) {
            //previous waypoints
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = waypoints_gap;
          double target_y = spl(target_x);
          double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
          double add_x = 0.0;

          //50 waypoints
          for (int i=0; i<50-prev_path_size; i++) {
            ref_vel += new_vel;

            //avoid driving over the speed limit!
            if (ref_vel > MAX_VEL)
              ref_vel = MAX_VEL;

            //0.02 seconds to the next waypoint | convenrting the ref vel from MPH to m/s
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double point_x = add_x + target_x / N;
            double point_y = spl(target_x);

            add_x = point_x;
            double new_ref_x = point_x;
            double new_ref_y = point_y;

            //convert local coordinates to global coordinates
            point_x = new_ref_x * cos(ref_yaw) - new_ref_y * sin(ref_yaw);
            point_y = new_ref_x * sin(ref_yaw) - new_ref_y * cos(ref_yaw);

            point_x += new_ref_x;
            point_y += new_ref_y;

            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
