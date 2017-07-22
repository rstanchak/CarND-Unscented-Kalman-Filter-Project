#include "ukf.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing class UKF.
class UKFTest : public ::testing::Test {
 protected:
	 UKF ukf;

  UKFTest() {
  }

  virtual ~UKFTest() {
  }

  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

  void testProcessRadar( int timestamp_s, double rho, double phi, double phi_dot, double expected_x, double expected_y) {
    MeasurementPackage m = { .timestamp_=timestamp_s*1000000, .sensor_type_=MeasurementPackage::RADAR };
    m.raw_measurements_ = Eigen::VectorXd(3);
    m.raw_measurements_ << rho, phi, phi_dot;
    ukf.ProcessRadar(m);
    std::cout<<ukf.P_<<std::endl;
    EXPECT_LT(fabs(expected_x-ukf.x_(0)), 1e-10);
    EXPECT_LT(fabs(expected_y-ukf.x_(1)), 1e-10);
  }
  void testProcessLidar( int timestamp_s, double x, double y, double expected_x, double expected_y) {
    MeasurementPackage m = { .timestamp_=timestamp_s*1000000, .sensor_type_=MeasurementPackage::RADAR };
    m.raw_measurements_ = Eigen::VectorXd(2);
    m.raw_measurements_ << x, y;
    ukf.ProcessLidar(m);
    std::cout<<ukf.P_<<std::endl;
    EXPECT_LT(fabs(expected_x-ukf.x_(0)), 1e-10);
    EXPECT_LT(fabs(expected_y-ukf.x_(1)), 1e-10);
  }
};

TEST_F(UKFTest, MethodProcessRadarInit0deg) {
	testProcessRadar(0, 1, 0, 0, 1, 0);
	testProcessRadar(1, 1, 0, 0, 1, 0);
}
TEST_F(UKFTest, MethodProcessRadarInit90deg) {
	testProcessRadar(0, 1, M_PI/2, 0, 0, 1);
	testProcessRadar(1, 1, M_PI/2, 0, 0, 1);
}
TEST_F(UKFTest, MethodProcessRadarInit270deg) {
	testProcessRadar(0, 1, -M_PI/2, 0, 0, -1);
	testProcessRadar(1, 1, -M_PI/2, 0, 0, -1);
}
TEST_F(UKFTest, MethodProcessRadarInit180deg) {
	testProcessRadar(0, 1, M_PI, 0, -1, 0);
	testProcessRadar(1, 1, M_PI, 0, -1, 0);
}
TEST_F(UKFTest, MethodProcessLidarInit0deg) {
	testProcessLidar(0, 1, 0, 1, 0);
	testProcessLidar(1, 1, 0, 1, 0);
}
TEST_F(UKFTest, MethodProcessLidarInit90deg) {
	testProcessLidar(0, 0, 1, 0, 1);
	testProcessLidar(1, 0, 1, 0, 1);
}
TEST_F(UKFTest, MethodProcessLidarInit180deg) {
	testProcessLidar(0, -1, 0, -1, 0);
	testProcessLidar(1, -1, 0, -1, 0);
}
TEST_F(UKFTest, MethodProcessLidarInit270deg) {
	testProcessLidar(0, 0, -1, 0, -1);
	testProcessLidar(1, 0, -1, 0, -1);
}

}  // namespace
