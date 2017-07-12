#include <iostream>
#include <math.h>
#include "ukf.h"
#include "tools.h"

using namespace std;

int main(int argc, char* argv[]) { 
  // Create a Kalman Filter instance
  UKF ukf;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  ifstream f(argv[1]);
  string sensor_measurement;

  while(getline(f, sensor_measurement))
  {
      MeasurementPackage meas_package;
      istringstream iss( sensor_measurement );
      long long timestamp;
	  float rho=0;
	  float theta=0;
	  float rho_dot=0;

      // reads first element from the current line
      string sensor_type;
      iss >> sensor_type;

      if (sensor_type.compare("L") == 0) {
          meas_package.sensor_type_ = MeasurementPackage::LASER;
          meas_package.raw_measurements_ = VectorXd(2);
          float px;
          float py;
          iss >> px;
          iss >> py;
		  rho = sqrt(px*px+py*py);
		  theta = atan2(py,px);
		  rho_dot = 0;
          meas_package.raw_measurements_ << px, py;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
      } else if (sensor_type.compare("R") == 0) {

          meas_package.sensor_type_ = MeasurementPackage::RADAR;
          meas_package.raw_measurements_ = VectorXd(3);
          iss >> rho;
          iss >> theta;
          iss >> rho_dot;
          meas_package.raw_measurements_ << rho,theta, rho_dot;
          iss >> timestamp;
          meas_package.timestamp_ = timestamp;
      }
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      VectorXd gt_values(4);
      gt_values(0) = x_gt;
      gt_values(1) = y_gt; 
      gt_values(2) = vx_gt;
      gt_values(3) = vy_gt;
      ground_truth.push_back(gt_values);

      //Call ProcessMeasurment(meas_package) for Kalman filter
      ukf.ProcessMeasurement(meas_package);    	  

      //Push the current estimated x,y positon from the Kalman filter's state vector

      VectorXd estimate(4);

      double p_x = ukf.x_(0);
      double p_y = ukf.x_(1);
      double v  = ukf.x_(2);
      double yaw = ukf.x_(3);

      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      std::cout<<"Z,"<<rho<<","<<theta<<","<<rho_dot<<","<<rho*cos(theta)<<","<<rho*sin(theta);
      std::cout<<",T,"<<sqrt(x_gt*x_gt+y_gt*y_gt)<<","<<atan2(y_gt, x_gt)<<","<<(vx_gt*x_gt + vy_gt*y_gt)/sqrt(x_gt*x_gt+y_gt*y_gt)<<","<<x_gt<<","<<y_gt;
      std::cout<<",X,"<<sqrt(p_x*p_x+p_y*p_y)<<","<<atan2(p_y, p_x)<<","<<(v1*p_x+v2*p_y)/sqrt(p_x*p_x+p_y*p_y)<<","<<p_x<<","<<p_y<<std::endl;
      estimate(0) = p_x;
      estimate(1) = p_y;
      estimate(2) = v1;
      estimate(3) = v2;

      estimations.push_back(estimate);

      VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
      std::cout<<"RMSE "<<RMSE<<std::endl;
  }
}























































































