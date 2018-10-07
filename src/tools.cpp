#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() = default;

Tools::~Tools() = default;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Initialize RMSE to 0
  VectorXd rmse(4);
  rmse.fill(0.0);

  // Validate input vector sizes
  size_t n = estimations.size();
  if (n == 0 || ground_truth.size() != n) {
    cout << "Invalid input vector sizes" << endl;
    return rmse;
  }

  // Calculate the RMSE here.
  // Accumulate residuals
  for (int i = 0; i < n; ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse /= n;

  // Calculate the square root
  rmse = rmse.array().sqrt();

  return rmse;
}