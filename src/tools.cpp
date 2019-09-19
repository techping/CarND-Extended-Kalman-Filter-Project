#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  for (int i = 0; i < estimations.size(); ++i) {
    rmse += VectorXd((estimations[i] - ground_truth[i]).array() * (estimations[i] - ground_truth[i]).array());
  }
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3, 4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  if (px < 1e-8 && py < 1e-8) {
    std::cout << "[error] px and py are 0" << std::endl;
    return Hj;
  }
  Hj << px / sqrt(px * px + py * py), py / sqrt(px * px + py * py), 0, 0,
     -py / (px * px + py * py), px / (px * px + py * py), 0, 0,
     py * (vx * py - vy * px) / pow(px * px + py * py, 1.5), px * (vy * px - vx * py) / pow(px * px + py * py, 1.5), px / sqrt(px * px + py * py), py / sqrt(px * px + py * py);
  return Hj;
}
