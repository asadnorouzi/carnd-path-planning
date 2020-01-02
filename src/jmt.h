#ifndef JMT_H
#define JMT_H

#include <vector>
#include <cmath>
#include "planner.h"
#include "helpers.h"

using namespace std;

class JMT {
public:
  JMT();
  ~JMT() {};

  void create_trajectory(Planner behavior, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int prev_path_size, double car_s, double car_x, double car_y, double car_yaw, int lane, vector<double>& next_x_vals, vector<double>& next_y_vals, double& vel);

  vector<double> waypoints_x_;
  vector<double> waypoints_y_;

  double ref_x_;
  double ref_y_;
  double ref_yaw_;

private:
  int prev_path_size_;
  Helper h_;
};

#endif
