#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
        const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4, 1);
  rmse << 0, 0, 0, 0;
  for(size_t i = 0; i<estimations.size(); ++i)
  {   
      VectorXd c = estimations[i]-ground_truth[i];
      rmse = rmse.array() + c.array()*c.array();
  }
  rmse = rmse.array()/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}
