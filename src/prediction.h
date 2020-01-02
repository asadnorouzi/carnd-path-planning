#ifndef PREDICTION_H
#define PREDICTION_H

#include <vector>

using namespace std;

class Prediction {
public:
  Prediction(int lane);
  ~Prediction() {};

  void predict(vector<vector<double>> sensor_fusion, double car_s, int prev_path_size);

  bool is_car_ahead_;
  bool is_car_left_;
  bool is_car_right_;
private:
  int my_lane_;
};

#endif
