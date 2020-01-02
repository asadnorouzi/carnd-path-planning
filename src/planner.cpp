#include "planner.h"
#include "params.h"

Planner::Planner()
{
  add_vel_ = 0.0;
}

void Planner::plan(Prediction pred, int& lane, double vel)
{
  if (pred.is_car_ahead_) {
    if (!pred.is_car_left_ && lane != 0)
     //move to left lane
      lane--;
    else if (!pred.is_car_right_ && lane != 2)
      //move to right lane
      lane++;
    else
      //do not change lane!
      add_vel_ -= params::MAX_ACC;
  }
  else {
    //no car ahead
    if (vel < params::MAX_VEL)
      add_vel_ += params::MAX_ACC;
  }
}
