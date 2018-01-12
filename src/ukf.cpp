#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
static const double XY_THRESH = 1e-4;

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma state dimension
  n_sig_ = 2 * n_aug_ + 1;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // initialize predicted sigma points matrix
  Xsig_pred_.fill(0.0);

  // initialize state vector: [px, py, v, yaw_angle, yaw_rate]
  x_ << 1, 1, 0, 0, 0;

  // initialize state covariance matrix P
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // create vector for weights
  weights_ = VectorXd(n_sig_);
  weights_(0) = lambda_/(lambda_+n_aug_);;
  for (int i=1; i<n_sig_; i++) {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  // initialize radar measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

  // initialize laser measurement noise covariance matrix
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

  previous_timestamp_ = 0;
}

UKF::~UKF() {}

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
  if (!is_initialized_) {
    double x = 0.0,
        y = 0.0,
        v = 0.0,
        yaw_angle = 0.0,
        yaw_rate = 0.0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // range (radial distance from origin to the tracking object such as pedestrian, car, etc.)
      double rho = meas_package.raw_measurements_[0];
      // bearing (angle between rho and x)
      double phi = meas_package.raw_measurements_[1];
      // radial velocity (change of rho - range rate)
      double rhoDot = meas_package.raw_measurements_[2];

      // Convert radar from polar to cartesian coordinates
      x = rho * cos(phi);
      y = rho * sin(phi);
      double vx = rhoDot * cos(phi);
      double vy = rhoDot * sin(phi);

      // find v (hypotenuse) using vx and vy
      v = sqrt(pow(vx, 2) + pow(vy, 2));
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x = meas_package.raw_measurements_[0],
          y = meas_package.raw_measurements_[1];
    }

    if (fabs(x) < XY_THRESH) {
      x = XY_THRESH;
    }
    if (fabs(y) < XY_THRESH) {
      y = XY_THRESH;
    }

    x_ << x, y, v, yaw_angle, yaw_rate;

    // done initializing, no need to predict or update
    previous_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0; // dt - expressed in seconds
  previous_timestamp_ = meas_package.timestamp_;

  // run prediction
  Prediction(dt);
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // Radar updates
    UpdateRadar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // Laser updates
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
