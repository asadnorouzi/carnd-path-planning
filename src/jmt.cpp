#include "jmt.h"
#include "params.h"
#include "spline.h"

JMT::JMT()
{
  prev_path_size_ = 0;
  ref_x_ = 0.0;
  ref_y_ = 0.0;
  ref_yaw_ = 0.0;
}

void JMT::create_trajectory(Planner behavior, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, int prev_path_size, double car_s, double car_x, double car_y, double car_yaw, int lane, vector<double>& next_x_vals, vector<double>& next_y_vals, double& vel)
{
  prev_path_size_ = prev_path_size;
  ref_x_ = car_x;
  ref_y_ = car_y;
  ref_yaw_ = h_.deg2rad(car_yaw);

  if (prev_path_size_ < 2) {
    //we use car location as reference
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    waypoints_x_.push_back(prev_car_x);
    waypoints_x_.push_back(car_x);
    waypoints_y_.push_back(prev_car_y);
    waypoints_y_.push_back(car_y);
  } else {
    //we use previous path as reference
    ref_x_ = previous_path_x[prev_path_size_-1];
    ref_y_ = previous_path_y[prev_path_size_-1];

    double prev_ref_x = previous_path_x[prev_path_size_-2];
    double prev_ref_y = previous_path_y[prev_path_size_-2];

    ref_yaw_ = atan2(ref_y_ - prev_ref_y, ref_x_ - prev_ref_x);

    waypoints_x_.push_back(prev_ref_x);
    waypoints_x_.push_back(ref_x_);
    waypoints_y_.push_back(prev_ref_y);
    waypoints_y_.push_back(ref_y_);
  }

  //calculate waypoints such that they're 30m apart.
  vector<double> next_waypoint_0 = h_.getXY(car_s + params::WAYPOINTS_GAP,
                                         (2 + 4 * lane), map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);
  vector<double> next_waypoint_1 = h_.getXY(car_s + params::WAYPOINTS_GAP*2,
                                         (2 + 4 * lane), map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);
  vector<double> next_waypoint_2 = h_.getXY(car_s + params::WAYPOINTS_GAP*3,
                                         (2 + 4 * lane), map_waypoints_s,
                                         map_waypoints_x, map_waypoints_y);

  waypoints_x_.push_back(next_waypoint_0[0]);
  waypoints_x_.push_back(next_waypoint_1[0]);
  waypoints_x_.push_back(next_waypoint_2[0]);
  waypoints_y_.push_back(next_waypoint_0[1]);
  waypoints_y_.push_back(next_waypoint_1[1]);
  waypoints_y_.push_back(next_waypoint_2[1]);

  for (unsigned int i=0; i<waypoints_x_.size(); i++) {
    //shifting the waypoints coordinates to the car's local coordinate
    double shift_x = waypoints_x_[i] - ref_x_;
    double shift_y = waypoints_y_[i] - ref_y_;

    waypoints_x_[i] = shift_x * cos(0 - ref_yaw_) - shift_y * sin(0 - ref_yaw_);
    waypoints_y_[i] = shift_x * sin(0 - ref_yaw_) + shift_y * cos(0 - ref_yaw_);
  }

  //create the spline
  tk::spline spl;
  spl.set_points(waypoints_x_, waypoints_y_);

  for (int i=0; i<prev_path_size_; i++) {
    //previous waypoints
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = (double)params::WAYPOINTS_GAP;
  double target_y = spl(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double add_x = 0.0;

  //50 waypoints
  for (int i=0; i<=50-prev_path_size_; i++) {
    vel += behavior.add_vel_;

    //avoid driving over the speed limit!
    if (vel > params::MAX_VEL)
      vel = params::MAX_VEL;

    //0.02 seconds to the next waypoint | convenrting the ref vel from MPH to m/s
    double N = target_dist / (0.02 * vel / 2.24);
    double point_x = add_x + target_x / N;
    double point_y = spl(point_x);

    add_x = point_x;
    double new_ref_x = point_x;
    double new_ref_y = point_y;

    //convert local coordinates to global coordinates
    point_x = new_ref_x * cos(ref_yaw_) - new_ref_y * sin(ref_yaw_);
    point_y = new_ref_x * sin(ref_yaw_) + new_ref_y * cos(ref_yaw_);

    point_x += ref_x_;
    point_y += ref_y_;

    next_x_vals.push_back(point_x);
    next_y_vals.push_back(point_y);
  }
}
