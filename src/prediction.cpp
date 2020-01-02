#include "prediction.h"
#include "params.h"
#include<cmath>

using namespace std;

Prediction::Prediction(int lane)
{
  is_car_ahead_ = false;
  is_car_left_ = false;
  is_car_right_ = false;

  my_lane_ = lane;
}

void Prediction::predict(vector<vector<double>> sensor_fusion, double car_s, int prev_path_size)
{
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

    if (other_car_lane == my_lane_)
      is_car_ahead_ |= other_car_s > car_s && other_car_s - car_s < params::GAP_BUF;
    else if (other_car_lane - my_lane_ == -1)
      is_car_left_ |= car_s - other_car_s < params::GAP_BUF && other_car_s - car_s < params::GAP_BUF;
    else if (other_car_lane - my_lane_ == 1)
      is_car_right_ |= car_s - other_car_s < params::GAP_BUF && other_car_s - car_s < params::GAP_BUF;
  }
}
