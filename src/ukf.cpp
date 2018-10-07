#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  P_(0, 0) = 1.0;
  P_(1, 1) = 1.0;
  P_(2, 2) = 10.0;
  P_(3, 3) = 1.0;
  P_(4, 4) = 1.0;

  // Initialize sigma prediction matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.75;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // measurement matrix
  H_ = MatrixXd(2, n_x_);
  H_.fill(0.0);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  // Measurement covariance matrix (Lidar)
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0.0,
      0.0, std_laspy_*std_laspy_;

  // Measurement covariance matrix (Radar)
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0.0, 0.0,
      0.0, std_radphi_*std_radphi_, 0.0,
      0.0, 0.0, std_radrd_*std_radrd_;
}

UKF::~UKF() = default;

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // Initialize with the first measurement
  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Laser measurement only provides x, y locations.
      // Assume velocity, yaw and yaw rate as 0.
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // A single radar measurement doesn't provide direction and yaw details.
      // Assume the object is moving straight away.
      double r = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double r_dot = meas_package.raw_measurements_(2);
      x_(0) = r * cos(phi);
      x_(1) = r * sin(phi);
      x_(2) = r_dot;
    } else {
      cout << "Failed to initialize the UKF model because the sensor type (" << meas_package.sensor_type_<<  ") is unknown." << endl;
      return;
    }
    // Set the measurement timestamp (in microsecond)
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*******************
   * Prediction
   *******************/
  // Compute time difference since previous measurement, and update the timestamp tracking
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(dt);

  /*******************
   * Update
   *******************/
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
   UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
   UpdateRadar(meas_package);
  } else {
   cout << "Failed to perfrom measurement update because the sensor type (" << meas_package.sensor_type_<<  ") is unknown." << endl;
   return;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**************************
   * Generate sigma points
   **************************/
  // Create augmented state vector, x_aug.
  // The means of acceleration and yaw acceleration noises are zero.
  VectorXd x_aug(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // Create augmented state covariance, P_aug.
  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  MatrixXd Q(2, 2);
  Q << std_a_*std_a_, 0.0,
      0.0, std_yawdd_*std_yawdd_;
  P_aug.block(n_x_, n_x_, 2, 2) = Q;

  // Compute square root of the state covariance matrix
  MatrixXd P_sqrt = P_aug.llt().matrixL();

  // Compute sigma points
  MatrixXd x_sig(n_aug_, 2*n_aug_ + 1);
  x_sig.col(0) = x_aug;
  const size_t offset_p = 1;
  const size_t offset_n = n_aug_ + 1;
  for (size_t i = 0; i < n_aug_; ++i) {
    x_sig.col(offset_p + i) = x_aug + sqrt(lambda_ + n_aug_) * P_sqrt.col(i);
    x_sig.col(offset_n + i) = x_aug - sqrt(lambda_ + n_aug_) * P_sqrt.col(i);
  }

  /**************************
   * Predict sigma points
   **************************/
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    double px = x_sig(0, i);
    double py = x_sig(1, i);
    double v = x_sig(2, i);
    double yaw = x_sig(3, i);
    double yaw_dot = x_sig(4, i);
    double a_noise = x_sig(5, i);
    double yaw_ddot_noise = x_sig(6, i);
    double delta_t2 = delta_t * delta_t;

    // Initialize the predicted state to the previous sigma points
    Xsig_pred_.col(i) << px, py, v, yaw, yaw_dot;
    // Add deterministic process contribution
    if (fabs(yaw_dot) < 0.001) {
      // Assume 0 yaw rate (ie. straight movement)
      Xsig_pred_(0, i) += v * cos(yaw) * delta_t;
      Xsig_pred_(1, i) += v * sin(yaw) * delta_t;
    } else {
      Xsig_pred_(0, i) +=  (v/yaw_dot) * (sin(yaw+yaw_dot*delta_t) - sin(yaw));
      Xsig_pred_(1, i) +=  (v/yaw_dot) * (-cos(yaw+yaw_dot*delta_t) + cos(yaw));
      Xsig_pred_(3, i) += yaw_dot * delta_t;
    }
    // Add process noise contribution
    Xsig_pred_(0, i) += 0.5 * delta_t2 * cos(yaw) * a_noise;
    Xsig_pred_(1, i) += 0.5 * delta_t2 * sin(yaw) * a_noise;
    Xsig_pred_(2, i) += delta_t * a_noise;
    Xsig_pred_(3, i) += 0.5 * delta_t2 * yaw_ddot_noise;
    Xsig_pred_(4, i) += delta_t * yaw_ddot_noise;
  }

  /*********************************
   * Predict mean and covariance
   *********************************/
  // Predict mean
  x_.fill(0.0);
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  // Predict covariance
  P_.fill(0.0);
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    VectorXd Xi_diff = Xsig_pred_.col(i) - x_;
    Xi_diff(3) = normalizeAngle(Xi_diff(3));
    P_ += weights_(i) * (Xi_diff * Xi_diff.transpose());
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  // Return if Ladar is not used
  if (!use_laser_)
    return;

  // Update the state by using Kalman Filter equations
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd P_Ht = P_ * Ht;
  MatrixXd S = H_ * P_Ht + R_lidar_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_Ht * Si;
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
  P_ = (I - K * H_) * P_;

  /*****************************
   * Perform consistency check
   *****************************/
  nis_ = y.transpose() * Si * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  // Return if Radar is not used
  if (!use_radar_)
    return;

  VectorXd z = meas_package.raw_measurements_;
  const size_t n_z = n_z_radar_;

  /************************
   * Predict Measurement
   ************************/
  // Measurement prediction
  VectorXd z_pred(n_z);
  z_pred.fill(0.0);

  // Estimate the measured state for the predicted sigma points
  MatrixXd Zsig_pred(n_z, 2*n_aug_ + 1);
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    // Transform sigma point predictions to measurement space
    double rho = sqrt(px*px + py*py);
    Zsig_pred.col(i) << rho, atan2(py, px), (px*cos(yaw)*v + py*sin(yaw)*v) / rho;
  }

  // Compute measurement predictions through the weighted sum
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig_pred.col(i);
  }

  /**********************************************
   * Update state using a new Radar measurement
   **********************************************/
  // Calculate covariance for measurement predictions
  MatrixXd S(n_z, n_z);
  S.fill(0.0);
  for (size_t i = 0; i < 2*n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    z_diff(1) = normalizeAngle(z_diff(1));
    S += weights_(i) * (z_diff * z_diff.transpose());
  }
  S += R_radar_;

  // Calculate cross-correlation b/w sigma points in state space and measurement space
  MatrixXd T(n_x_, n_z);
  T.fill(0.0);
  for (size_t i = 0; i < 2*n_aug_ + 1; ++i) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = normalizeAngle(x_diff(3));
    VectorXd z_diff = Zsig_pred.col(i) - z_pred;
    z_diff(1) = normalizeAngle(z_diff(1));
    T += weights_(i) * (x_diff * z_diff.transpose());
  }

  // Calculate the updated state and covariance matrix
  MatrixXd Si = S.inverse();
  MatrixXd K = T * Si;
  VectorXd z_error = z - z_pred;
  z_error(1) = normalizeAngle(z_error(1));
  x_ = x_ + K * z_error;
  P_ = P_ - (K * S * K.transpose());

  /*****************************
   * Perform consistency check
   *****************************/
  nis_ = z_error.transpose() * Si * z_error;
}

/**
 * Normalize the input angle in radian to [-PI, PI)
 * @param rad input angle in radian
 * @return normalized angle in radian
 */
double UKF::normalizeAngle(double rad) {
  // First, check if the angle is already in the desired range [-PI, PI)
  if (!(rad >= -M_PI && rad < M_PI)) {
    // 1. Add PI to the given angle - the desired range is transitioned to [0, 2*PI)
    // 2. Using fmod to ensure the angle is in the desired range without change the actual angle
    // 3. Subtract PI to transition the result back to [-PI, PI) range
    rad = fmod(rad + M_PI, 2*M_PI) - M_PI;
  }
  return rad;
}
