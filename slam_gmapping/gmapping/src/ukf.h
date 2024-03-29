#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include "tools.h"
#include <vector>
#include <string>
#include <fstream>

// use to read yaml
#include "yaml-cpp/yaml.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  //////////////////////////// add parameter ///////////////////////////
  ///* control sigma points separate degree
  double std_alpha_ ;

  ///* control sigma points separate degree
  double std_k_ ;

  ///* control separate degree of covarience
  double std_beta_ ;
  //////////////////////////////////////////////////////////////////////

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  // n_aug_ : 擴展向量的維度（狀態向量的維度加上雜訊向量的維度）
  int n_aug_;

  //Number of sigma points
  // n_sig_ : sigma點的數量, 通常設定為 2*n_aug_ + 1, 其中包含中心點
  int n_sig_;

  ///* Sigma point spreading parameter
  double lambda_;

  //Process noise matrix
  MatrixXd Q_;

  //Create tools object to use methods
  Tools tool_;

  //Create laser sensor transformation matrix
  MatrixXd H_laser_;

  //Create  measurement uncertainty matrix for laser and radar
  MatrixXd R_laser_;
  MatrixXd R_radar_;

  //Declare NIS variables
  double NIS_radar_;
  double NIS_laser_;

  // residual of UKF
  VectorXd y_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  bool ProcessMeasurement(MeasurementPackage &meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  // set ORBSLAM2 and PLICP param
  void SetUKFParam(int a, std::string path);
};

#endif /* UKF_H */

