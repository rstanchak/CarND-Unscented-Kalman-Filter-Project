#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"

using std::cout;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/* helper function prototypes */
static void GenerateSigmaPoints(int n_x, double lambda, const VectorXd & x, const MatrixXd & P, MatrixXd * retval_Xsig);
static void AugmentedSigmaPoints(const VectorXd &x, const MatrixXd &P, const VectorXd & std_addl, double lambda, MatrixXd * retval_Xsig_aug);
static void SigmaPointPrediction(int n_x, const MatrixXd &Xsig_aug, double dt, MatrixXd* Xsig_out);
static void GenerateWeights(int n_aug, double lambda, VectorXd * retarg_weights);
static void PredictMeanAndCovariance(const MatrixXd & Xsig_pred, const VectorXd & weights, VectorXd* retarg_x, MatrixXd* retarg_P);
static void PredictRadarMeasurement(const MatrixXd & Xsig_pred, const VectorXd & weights, const MatrixXd & R, MatrixXd * Zsig_out, VectorXd* z_out, MatrixXd* S_out);
static void UpdateState(VectorXd & x, MatrixXd & P, const VectorXd & z, const MatrixXd & Xsig_pred, const VectorXd & weights, const MatrixXd & Zsig, const VectorXd & z_pred, const MatrixXd & S);
static double normalize_angle(double theta);

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = n_aug_ - 3;
  P_.diagonal() << 1., 1., 1000., 1000., 1000.;
  use_laser_ = false;
  GenerateWeights(n_aug_, lambda_, &weights_);

  R_rad_ = MatrixXd(3, 3);
  R_rad_.diagonal() << std_radr_*std_radr_, std_radphi_ * std_radphi_, std_radrd_*std_radrd_;
  R_las_ = MatrixXd(2, 2);
  R_las_.diagonal() << std_laspx_*std_laspx_, std_laspy_*std_laspy_;
}

UKF::~UKF() {
}

void UKF::ProcessLidar(MeasurementPackage meas_package) {
  double dt = (meas_package.timestamp_ - time_us_)/1000000.;

  // update current time
  time_us_ = meas_package.timestamp_;

  // initialize state from first measurement
  if(!is_initialized_) {
    x_ << meas_package.raw_measurements_[0], 
       meas_package.raw_measurements_[1], 0, 0, 0;
    is_initialized_ = true;
    return;
  }
  this->Prediction(dt);
  this->UpdateLidar(meas_package);
}

void UKF::ProcessRadar(MeasurementPackage meas_package) {
  double dt = (meas_package.timestamp_ - time_us_)/1000000.;
  double rho = meas_package.raw_measurements_[0];
  double phi = meas_package.raw_measurements_[1];

  // update current time
  time_us_ = meas_package.timestamp_;

  // initialize state from first measurement
  if(!is_initialized_) {
    x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    is_initialized_ = true;
    return;
  }
  this->Prediction(dt);
  this->UpdateRadar( meas_package );
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_package) {
  switch(measurement_package.sensor_type_)
  {
    case MeasurementPackage::LASER:
      if(!use_laser_) return;
      this->ProcessLidar( measurement_package );
      break;
    case MeasurementPackage::RADAR:
      if(!use_radar_) return;
      this->ProcessRadar( measurement_package );
      break;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  VectorXd std_addl = Eigen::Vector2d(std_a_, std_yawdd_);

  // Estimate the object's location. 
  AugmentedSigmaPoints( x_, P_, std_addl, lambda_, &Xsig_pred_ );
  SigmaPointPrediction( x_.size(), Xsig_pred_, delta_t, &Xsig_pred_ );
  PredictMeanAndCovariance(Xsig_pred_, weights_, &x_, &P_);
  
  // Predict sigma points, the state, and the state covariance matrix.
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
  MatrixXd S;
  VectorXd z_pred;
  MatrixXd Zsig;

  PredictRadarMeasurement( Xsig_pred_, weights_, R_rad_, &Zsig, &z_pred, &S);
  UpdateState( x_, P_, meas_package.raw_measurements_, Xsig_pred_, weights_, Zsig, z_pred, S );

  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

/**
 */
static void GenerateSigmaPoints(int n_x, double lambda, const VectorXd & x, const MatrixXd & P, MatrixXd * retval_Xsig) {

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  //calculate square root of P
  MatrixXd A = P.llt().matrixL();

  Xsig.colwise() = x;
  Xsig.block(0,1,5,5) += sqrt(n_x+lambda) * A;
  Xsig.block(0,6,5,5) -= sqrt(n_x+lambda) * A;

  //write result
  *retval_Xsig = Xsig;
}


static void AugmentedSigmaPoints( const VectorXd &x, const MatrixXd &P, const VectorXd & std_addl, double lambda, MatrixXd * retval_Xsig_aug) {

  // additional state dimensions in augmented state
  int n_x = x.size();
  int n_addl = std_addl.size();
  int n_aug = n_x + n_addl;

  // create augmented mean vector
  VectorXd x_aug(n_aug);

  // create augmented state covariance
  MatrixXd P_aug(n_aug, n_aug);

  // create sigma point matrix
  MatrixXd Xsig_aug(n_aug, 2 * n_aug + 1);

  // create augmented mean state
  x_aug.head(n_x) = x;
  x_aug.tail(n_addl).fill(0);

  // create augmented covariance matrix
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug.bottomRightCorner(n_addl, n_addl).diagonal() << std_addl.array().pow(2);

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.colwise() = x_aug;
  Xsig_aug.block(0, 1, n_aug, n_aug) += sqrt(n_aug+lambda) * A;
  Xsig_aug.block(0, 1+n_aug, n_aug, n_aug) -= sqrt(n_aug+lambda) * A;

  // write result
  *retval_Xsig_aug = Xsig_aug;
}


static void SigmaPointPrediction(int n_x, const MatrixXd &Xsig_aug, double dt, MatrixXd* Xsig_out) {

  int n_aug = Xsig_aug.rows();

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //predict sigma points
  for(int i=0; i<(2*n_aug+1); ++i)
  {
    double v = Xsig_aug(2, i);
    double psi = Xsig_aug(3, i);
    double psi_dot = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_psi = Xsig_aug(6, i);

    // avoid division by zero
    if(fabs(psi_dot) > 0.001)
    {
      Xsig_pred.col(i) <<
        v/psi_dot*(sin(psi+psi_dot*dt)-sin(psi)) + 0.5*dt*dt*cos(psi)*nu_a,
        v/psi_dot*(-cos(psi+psi_dot*dt)+cos(psi)) + 0.5*dt*dt*sin(psi)*nu_a,
        dt * nu_a,
        psi_dot*dt + 0.5*dt*dt*nu_psi,
        dt * nu_psi;
    }
    else
    {
      Xsig_pred.col(i) <<
        v*cos(psi)*dt + 0.5*dt*dt*cos(psi)*nu_a,
        v*sin(psi)*dt + 0.5*dt*dt*sin(psi)*nu_a,
        dt * nu_a,
        0.5*dt*dt*nu_psi,
        dt * nu_psi;
    }
    Xsig_pred.col(i) += Xsig_aug.block(0,i,n_x,1);
  }

  //write result
  *Xsig_out = Xsig_pred;

}

static void GenerateWeights( int n_aug, double lambda, VectorXd * retarg_weights ) {
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  
  //set weights
  // w_1 = \lambda / ( \lambda + n_aug )
  // w_i = 1 / 2*( \lambda + n_aug
  weights(0) = lambda/(lambda + n_aug);
  weights.tail(2*n_aug).fill(1./(2*(lambda+n_aug)));
  *retarg_weights = weights;
}

static void PredictMeanAndCovariance(const MatrixXd & Xsig_pred, const VectorXd & weights, VectorXd* retarg_x, MatrixXd* retarg_P) {
  int n_aug = (Xsig_pred.cols()-1)/2;
  int n_x = Xsig_pred.rows();

  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);
 
  //predict state mean
  // x = sum_n ( X_i * w_i )
  x = Xsig_pred * weights;

  //predict state covariance matrix
  P = MatrixXd::Zero(n_x, n_x);

  for(int i=0; i<(2*n_aug+1); ++i)
  {
    MatrixXd y = Xsig_pred.col(i) - x;
    y(3) = normalize_angle(y(3));
    P += weights(i) * y * y.transpose();
  }

  //write result
  *retarg_x = x;
  *retarg_P = P;
}


static void PredictRadarMeasurement(const MatrixXd & Xsig_pred, const VectorXd & weights, const MatrixXd & R, MatrixXd * Zsig_out, VectorXd* z_out, MatrixXd* S_out) {

  //set state dimension
  int n_x = Xsig_pred.rows();

  //set augmented dimension
  int n_aug = (Xsig_pred.cols()-1)/2;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = R.rows();

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //transform sigma points into measurement space
  for(int i=0; i<(2*n_aug+1); ++i)
  {
    double x = Xsig_pred(0,i);
    double y = Xsig_pred(1,i);
    double v = Xsig_pred(2,i);
    double psi = Xsig_pred(3, i);
    // rho
    double rho = sqrt( x*x + y*y );
    Zsig(0,i) = rho;
    // phi
    Zsig(1,i) = atan2( y, x );
    // rho_dot
    Zsig(2,i) = (x * cos( psi )*v + y * sin( psi )*v)/rho;
  }
  //calculate mean predicted measurement
  z_pred = Zsig * weights;
  //calculate measurement covariance matrix S
  S.fill(0.);
  for(int i=0; i<(2*n_aug+1); ++i)
  {
    MatrixXd z_diff = Zsig.col(i)-z_pred;
    //angle normalization
    z_diff(1) = normalize_angle(z_diff(1));
    S += weights(i) * z_diff * z_diff.transpose();
  }
  S += R;

  //write result
  *Zsig_out = Zsig;
  *z_out = z_pred;
  *S_out = S;
}

static double normalize_angle(double theta) {
	return atan2( sin(theta), cos(theta) );
}

void UpdateState( VectorXd & x, MatrixXd & P, const VectorXd & z, const MatrixXd & Xsig_pred, const VectorXd & weights, const MatrixXd & Zsig, const VectorXd & z_pred, const MatrixXd & S ) {

  //set state dimension
  int n_x = Xsig_pred.rows();

  //set augmented dimension
  int n_aug = (Xsig_pred.cols()-1)/2;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  //calculate cross correlation matrix
  Tc.fill(0);
  for(int i=0; i<(2*n_aug+1); ++i)
  {
    MatrixXd Xdiff = Xsig_pred.col(i)-x;
    MatrixXd Zdiff = Zsig.col(i)-z_pred;
    Xdiff(3) = normalize_angle(Xdiff(3));
    Zdiff(1) = normalize_angle(Zdiff(1));
    
    Tc += weights(i)*Xdiff*Zdiff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //update state mean and covariance matrix
  MatrixXd z_diff = z-z_pred;
  z_diff(1) = normalize_angle(z_diff(1));
  x = x + K*z_diff;
  P = P - K*S*K.transpose();
}

void test_AugmentedSigmaPoints()
{
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  //Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  VectorXd std(2);
  std << std_a, std_yawdd;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  //create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  MatrixXd Xsig_aug;

  AugmentedSigmaPoints( x, P, std, lambda, &Xsig_aug );

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
}


void test_PredictMeanAndCovariance() {

  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  VectorXd weights;
  GenerateWeights( n_aug, lambda, &weights );

  //create vector for predicted state
  VectorXd x = VectorXd(n_x);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);

  PredictMeanAndCovariance( Xsig_pred, weights, &x, &P );

  //print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P << std::endl;
}

void test_SigmaPointPrediction()
{
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  double delta_t = 0.1; //time diff in sec

  //create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  MatrixXd Xsig_pred;
  SigmaPointPrediction(n_x, Xsig_aug, delta_t, &Xsig_pred);
  //print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

}

void test_PredictRadarMeasurement()
{
  VectorXd z_pred;
  MatrixXd S;
  MatrixXd Zsig;
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  int lambda = 3-n_aug;

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  VectorXd weights;
  GenerateWeights( n_aug, lambda, &weights );

  MatrixXd R(3,3);
  R.diagonal() << std_radr*std_radr, std_radphi*std_radphi, std_radrd*std_radrd;
 
  //create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  PredictRadarMeasurement(Xsig_pred, weights, R, &Zsig, &z_pred, &S);

  /*******************************************************************************
   * Student part end
   ******************************************************************************/

  //print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

}

void test_UpdateState()
{
  int n_x = 5;
  int n_aug = 7;
  int n_z = 3;
  double lambda = 3 - n_aug;

  //create example matrix with predicted sigma points in state space
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  //create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  //create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);
  P <<
  0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

  VectorXd weights;
  GenerateWeights( n_aug, lambda, &weights );

  //create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig <<
      6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
     0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
      2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred <<
      6.12155,
     0.245993,
      2.10313;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
      5.9214,   //rho in m
      0.2187,   //phi in rad
      2.0062;   //rho_dot in m/s
  UpdateState( x, P, z, Xsig_pred, weights, Zsig, z_pred, S);
  //print result
  std::cout << "Updated state x: " << std::endl << x << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

}
