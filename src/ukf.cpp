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
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;


  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
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
  
	
	

	n_x_ = 5;
	n_aug_ = n_x_ + 2;
	lambda_ = 3 - n_aug_;
	
	n_sigma_points = 2 * n_aug_ + 1;
	
	// initial state vector
  x_ = VectorXd(n_x_);
	x_ << 0.0, 0.0, 0.0, 0.0, 0.0;
	
  // initial covariance matrix
	// I noticed that the values used to init the covariance
	// impact the covergence of the matrix
	// with 1000 on v, yaw and yaw_rate, the algo diverges
  P_ = MatrixXd(n_x_, n_x_);
	P_ << 0.5, 0.0, 0.0, 0.0, 0.0,
				0.0, 0.5, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0;
	
	Xsig_pred_ = MatrixXd(n_x_, n_sigma_points);


	
	// Weights
	int denominator = n_aug_ + lambda_;
	weights_ = VectorXd(n_sigma_points);
	weights_(0) = lambda_ / denominator;
	
	for (int i = 1; i < n_sigma_points; ++i)
	{
		weights_(i) = 0.5 / denominator;
		
	}
	
	
	 // initializing matrices
  R_lidar = MatrixXd(2, 2);
  R_radar = MatrixXd(3, 3);
	  
  

  //measurement covariance matrix - laser
  R_lidar << std_laspx_*std_laspx_, 0.0,
							0.0, std_laspy_*std_laspy_;

  //measurement covariance matrix - radar
  R_radar << std_radr_*std_radr_, 0.0, 0.0,
							0.0, std_radphi_*std_radphi_, 0.0,
							0.0, 0.0, std_radrd_*std_radrd_;
							
							

	
	time_us_ = 0;
	
	nis_lidar = 0.0;
	nis_radar = 0.0;
  
}


UKF::~UKF() {}




static void angle_trunc(double& yaw)
{
  
	// from AI robotics
	// phi to be [-pi, pi] Learned it from AI for robotics
	static const double PI = 3.14159265358979323846264338327950;
  while (yaw > PI) {
    yaw -= 2.*PI;
  };
  while (yaw < -PI) {
    yaw += 2.*PI;
  };
    
}



/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
	/*  Initialization  */
	
	if (!is_initialized_)
	{
		cout << "Initialization " << endl;
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			
			double rho = meas_package.raw_measurements_(0);
			double yaw = meas_package.raw_measurements_(1);
			double rho_dot = meas_package.raw_measurements_(2);
			
			double vx = rho_dot*cos(yaw);
			double vy = rho_dot*sin(yaw);
			
			x_(0) = rho     * cos(yaw);
			x_(1) = rho 		* sin(yaw);						
			x_(2) = sqrt(vx*vx + vy*vy);
			x_(3) = 0.0;
			x_(4) = 0.0;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			
      x_(0) = meas_package.raw_measurements_(0);
			x_(1) = meas_package.raw_measurements_(1);
			x_(2) = 0.0;
			x_(3) = 0.0;
			x_(4) = 0.0;
    }
		
    time_us_ = meas_package.timestamp_;
		is_initialized_ = true;
		
		return ;
	}	
	
	/*  Prediction  */
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;
	
	if (dt > 0.001)
	{
		
		Prediction(dt);
	}
	
	
	/*  Update  */
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
	{
		UpdateRadar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
	{
		UpdateLidar(meas_package);
	}
		
	
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
		
	
	// sigma points
	MatrixXd X_sig_aug(n_aug_, n_sigma_points);
	unscentedTransform(X_sig_aug);
	
	double square_delta = delta_t * delta_t;
	
	// process : transform the sigma points
	for (int i = 0; i < n_sigma_points; ++i)
	{
		
		double px = 					X_sig_aug(0, i);
		double py = 					X_sig_aug(1, i); 
		double velocity = 		X_sig_aug(2, i);
		double yaw = 					X_sig_aug(3, i);
		double yaw_rate = 		X_sig_aug(4, i);
		double nu_velocity = 	X_sig_aug(5, i);
		double nu_yaw = 			X_sig_aug(6, i);
		
		
		// predicted sigma points after motion
		double px_t, py_t;
		
		
		if (fabs(yaw_rate) > 0.001) 
		{
			px_t = px + (velocity / yaw_rate) * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
			py_t = py + (velocity / yaw_rate) * (-cos(yaw + yaw_rate * delta_t) + cos(yaw));			
			
		}
		else 
		{
			// approximate model motion
			px_t = px + velocity * delta_t * cos(yaw);
			py_t = py + velocity * delta_t * sin(yaw);
		}
		
		double velocity_t = velocity;
		double yaw_t = yaw + yaw_rate * delta_t;
		double yaw_rate_t = yaw_rate;
		
		// process noise added
		px_t += 0.5 * square_delta * cos(yaw) * nu_velocity;
		py_t += 0.5 * square_delta * sin(yaw) * nu_velocity;
		velocity_t += delta_t * nu_velocity;
		yaw_t += 0.5 * square_delta * nu_yaw;
		yaw_rate_t += delta_t * nu_yaw;
		
		Xsig_pred_(0, i) = px_t;
		Xsig_pred_(1, i) = py_t; 
		Xsig_pred_(2, i) = velocity_t;
		Xsig_pred_(3, i) = yaw_t;
		Xsig_pred_(4, i) = yaw_rate_t;		
		
	}
	
	// Mean and covariance of augmented state 
	
	x_ = Xsig_pred_ * weights_;
	
	// reset the matrix 
	P_.fill(0.0);
	for (int i = 0 ; i < n_sigma_points; ++i)
	{
		VectorXd tmp = Xsig_pred_.col(i) - x_;
		angle_trunc(tmp(3));
		P_ = P_ + weights_(i) * tmp * tmp.transpose();
	}
	
	
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
 
	// state space to measurement space
	static int number_states_seen_lidar = 2;
	MatrixXd Z = Xsig_pred_.topLeftCorner(number_states_seen_lidar, n_sigma_points);
	
		
	VectorXd z(2);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);
	
	
	updateStep(Z, z, R_lidar, nis_lidar, false);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

	// state space to measurement space
	static int number_states_seen_radar = 3;
	
	MatrixXd Z(number_states_seen_radar, n_sigma_points);
	
	for (int i = 0; i < n_sigma_points; ++i)
	{			
		double px = 	Xsig_pred_(0, i);
		double py = 	Xsig_pred_(1, i);
		double v =  	Xsig_pred_(2, i);
		double yaw =  Xsig_pred_(3, i);
		
		double rho = sqrt(px*px + py*py);		
		double vx = v * cos(yaw);
		double vy = v * sin(yaw);
		
		// avoid zero division 
		if (rho < 0.001) 
		{
			rho = 0.001;
		}
		double rho_rate = (px*vx + py*vy) / rho;
		
				
		Z(0, i) = rho;
		Z(1, i) = atan2(py, px);
		Z(2, i) = rho_rate;
		
	}
	
	
	VectorXd z(3);
	z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package.raw_measurements_(2);
	
	updateStep(Z, z, R_radar, nis_radar, true);
	
}



/*
 * Perform Update step 
 * independently of the 
 * sensor 
 */

void UKF::updateStep(const MatrixXd& Z, const VectorXd& z, const MatrixXd& R, double& nis, bool is_radar)
{
	
	
	MatrixXd Z_mean = Z * weights_;
	
	MatrixXd S(z.rows(), z.rows());
	S.fill(0.0);
	for (int i = 0 ; i < n_sigma_points; ++i)
	{
		VectorXd diff_measure = Z.col(i) - Z_mean;
		if (is_radar)
		{			
			angle_trunc(diff_measure(1));
		}
		S = S + weights_(i) * diff_measure * diff_measure.transpose();
	}
	S = S + R;
	
	
	
	MatrixXd T(n_x_, z.rows());
	T.fill(0.0);

	for (int i = 0 ; i < n_sigma_points; ++i)
	{
		VectorXd diff_state = Xsig_pred_.col(i) - x_;
		VectorXd diff_measure = Z.col(i) - Z_mean;
		if (is_radar)
		{			
			angle_trunc(diff_state(3));		
			angle_trunc(diff_measure(1));
		}
		T = T + weights_(i) * diff_state * diff_measure.transpose();
	}
	
	MatrixXd K = T * S.inverse();

	
	VectorXd y_error = z - Z_mean;
	if (is_radar)
	{		
		angle_trunc(y_error(1));
	}
	
	// Update predicted state 
	x_ = x_ + K * y_error;
	P_ = P_ - K * S * K.transpose();
	
	// nis 
	nis = y_error.transpose() * S.inverse() * y_error;
	
	cout << "Kalman gain : " << K << endl;

	
}

/**
 * Perform the unscented transformation 
 */

void UKF::unscentedTransform(MatrixXd& X_sig_aug)
{
	
	
	
	VectorXd x_aug = VectorXd(n_aug_);
	
	x_aug.head(n_x_) = x_;
	
	// the two noises are zero mean
	x_aug(n_x_) = 0.0;
	x_aug(n_x_+1) = 0.0;
	
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

	
	
	
	MatrixXd Sqrt_of_P = P_aug.llt().matrixL();
	
	double coefficient = sqrt(n_aug_ + lambda_);
	
	X_sig_aug.col(0) = x_aug;
	
	for (int i = 0 ; i < n_aug_; ++i)
	{
		X_sig_aug.col(i+1) = x_aug + coefficient * Sqrt_of_P.col(i);
		X_sig_aug.col(i+n_aug_+1) = x_aug - coefficient * Sqrt_of_P.col(i);
	}
	
	
	
}
