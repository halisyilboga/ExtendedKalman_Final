#include <iostream>

#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

								 

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  VectorXd y = z - H_ * x_;
						  
  MatrixXd H_t = H_.transpose();
  
  MatrixXd PHt = P_ * H_t;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt *  S.inverse();
						 
						


  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}


void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
																  
											   
									 
  //kesinlestir
  MatrixXd hx(3,1);
									
						   

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px+py*py);
  float phi = atan2(py, px);

  if(rho < 0.0001){
      std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
      return;
  }
							   
								 
							
						 
						

  float rho_dot = (px*vx + py*vy)/rho;

  hx(0,0) = rho;
  hx(1,0) = phi;
  hx(2,0) = rho_dot;

  std::cout << "UpdateEKF () -  calculating ended" << std::endl;

  
  
  Eigen::VectorXd y = z - hx;

   
  if (phi  >  (PI)) {
     phi += 2*PI;
   } else if (phi > (PI)) {
     phi -= 2*PI;
   }
   
  while (y(1)>PI) {
    y(1) -= 2 * PI;
  }
  while (y(1) <-PI) {
    y(1) += 2 * PI;
  }
  


  MatrixXd H_t = H_.transpose();
  
  MatrixXd PHt = P_ * H_t;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_; 
}
