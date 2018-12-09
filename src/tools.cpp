#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

      // Initializing accuracy metrics with zeros
      VectorXd rmse(4);
      rmse << 0,0,0,0;

      if (estimations.size() < 1) {
        return rmse;
      }
      if (estimations.size() != ground_truth.size()) {
        return rmse;
      }

      for (int j=1; j < estimations.size(); ++j) {

        VectorXd residual = estimations[j] - ground_truth[j];

        residual = residual.array() * residual.array();
        rmse += residual;
      }


      rmse = rmse / estimations.size();


      rmse = rmse.array().sqrt();

      return rmse;


}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

    MatrixXd Hj(3,4);

    // Recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // Checking for zero division error
    float c1 = px*px + py*py;
    if (c1 < 0.00001) {
      cout << "Division by zero detected!" << "\n";
      px += 0.001;
      py += 0.001;
      c1 = px*px + py*py;
    }

    float c2 = sqrt(c1);
    float c3 = (c1 * c2)
    float vxpy_vypx = vx*py - vy*px;
    float l3 = vxpy_vypx / c3;

    Hj <<   px/c2,   py/c2,  0,               0,
            -py/c1,       px/c1,       0,               0,
            py*l3,     -px*l3,   px/c2,    py/c2;

    return Hj;

}
