#include "tools.h"
#include <iostream>



using std::vector;
using std::endl;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const vector<Eigen::VectorXd> &estimations,
                              const vector<Eigen::VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
  Eigen::VectorXd rmse(4);
  rmse << 0,0,0,0;

  //  * the estimation vector  should not be empty
  //  *  ground truth vector size and estimation vector size should be equal
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Ground truth vector or estimation vector is not valid" << endl;
    return rmse;
  }

   // accumulate squared diff
  for (unsigned int i=0; i < estimations.size(); ++i) {

    Eigen::VectorXd diff = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    diff = diff.array()*diff.array();
    rmse += diff;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state) {
 /**
    * Calculate a Jacobian here.
  */
  Eigen::MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

 
// check division by zero
  if (fabs(c1) < 0.001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  // the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
	  -(py/c1), (px/c1), 0, 0,
	  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}
