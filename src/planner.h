#ifndef PLANNER_H
#define PLANNER_H

#include "prediction.h"

class Planner {
public:
  Planner();
  ~Planner() {};

  void plan(Prediction pred, int& lane, double vel);

  double add_vel_;
};

#endif
