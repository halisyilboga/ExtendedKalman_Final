 #include "FusionEKF.h"
				  
					  
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

					
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  ekf_.Q_ = Eigen::MatrixXd(4, 4);


  //the initial transition matrix F_
  ekf_.F_ = Eigen::MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
	         0, 1, 0, 1,
	         0, 0, 1, 0,
	         0, 0, 0, 1;

  //measurement matrix
  H_laser_ = Eigen::MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;


  //state covariance matrix P
  ekf_.P_ = Eigen::MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  
	noise_ax_ = 9.0;
	noise_ay_ = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
																				
					
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            
			// double mi floatmi kesinlestir
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            double rho_dot = measurement_pack.raw_measurements_[2];
            
            //convert polar to cartesian coords
            double px = rho * cos(phi);
            double py = rho * sin(phi);
		
																									

            

      
            ekf_.x_ << px,py,0,0;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

    }
    // zero divide check. 
    if ( std::fabs(ekf_.x_(0)+ekf_.x_(1)) < 0.0001){
           ekf_.x_(0) = 0.0001;
           ekf_.x_(1) = 0.0001;
    }
  
     previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

																										 
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;

    
    //add t to F
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    
    //calculate t for Q
    float dt_2 = dt * dt;
					   
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;
						 


    
    // Acceleration deviation
     ekf_.Q_ << dt_4 / 4 * noise_ax_, 0 , dt_3 / 2 * noise_ax_, 0 ,
            0 , dt_4 / 4 * noise_ay_, 0 , dt_3 / 2 * noise_ay_,
            dt_3 / 2 * noise_ax_, 0 , dt_2 * noise_ax_, 0 ,
            0 , dt_3 / 2 * noise_ay_, 0 , dt_2 * noise_ay_; 
  


      //check if dt big enough.
      if ( dt > 1e-3 )
      {
         ekf_.Predict();
      }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

																   
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      //Radar updates 
											   
					   
      ekf_.R_ = R_radar_;

      //measurement matrix
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

      //Update
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

      } else {
        // Laser updates
        //measurement covariance matrix
        ekf_.R_ = R_laser_;

        //measurement matrix
        ekf_.H_ = H_laser_;

        //Update
        ekf_.Update(measurement_pack.raw_measurements_);
      }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
};

 
