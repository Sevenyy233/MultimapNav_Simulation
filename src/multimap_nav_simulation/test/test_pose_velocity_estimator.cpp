#include <cmath>

#include <gtest/gtest.h>

#include "multimap_nav_simulation/pose_velocity_estimator.hpp"

using multimap_nav_simulation::PoseVelocityEstimator;

tf2::Transform pose(double x, double y, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, 0.0));
}

TEST(PoseVelocity, TranslationIsExpressedInCurrentBaseFrame)
{
  PoseVelocityEstimator estimate(0.0);
  EXPECT_FALSE(estimate.update(pose(0, 0, M_PI_2), 1.0).valid);
  const auto result = estimate.update(pose(0, 0.04, M_PI_2), 1.1);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.linear.x(), 0.4, 1e-10);
  EXPECT_NEAR(result.linear.y(), 0.0, 1e-10);
  EXPECT_NEAR(result.angular.length(), 0.0, 1e-10);
}

TEST(PoseVelocity, QuaternionSignAndYawWrappingDoNotCreateSpikes)
{
  PoseVelocityEstimator estimate(0.0);
  estimate.update(pose(0, 0, M_PI-0.01), 1.0);
  auto next = pose(0, 0, -M_PI+0.01);
  const auto q = next.getRotation();
  next.setRotation(tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w()));
  const auto result = estimate.update(next, 1.1);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.angular.z(), 0.2, 1e-10);
}

TEST(PoseVelocity, ClockResetDuplicateAndLongGapResetTheDerivative)
{
  PoseVelocityEstimator estimate;
  estimate.update(pose(0, 0, 0), 1.0);
  EXPECT_TRUE(estimate.update(pose(0.04, 0, 0), 1.1).valid);
  EXPECT_FALSE(estimate.update(pose(0.04, 0, 0), 1.1).valid);
  EXPECT_FALSE(estimate.update(pose(4, 0, 0), 2.0).valid);
  EXPECT_FALSE(estimate.update(pose(0, 0, 0), 0.0).valid);
  EXPECT_NEAR(estimate.update(pose(0.04, 0, 0), 0.1).linear.x(), 0.4, 1e-10);
}

TEST(PoseVelocity, FilterSmoothsTheMeasuredVelocity)
{
  PoseVelocityEstimator estimate(0.2);
  estimate.update(pose(0, 0, 0), 1.0);
  estimate.update(pose(0.04, 0, 0), 1.1);
  const auto result = estimate.update(pose(0.12, 0, 0), 1.2);
  EXPECT_GT(result.linear.x(), 0.4);
  EXPECT_LT(result.linear.x(), 0.8);
}

TEST(NativeTwist, LeverArmRemovesVelocityOfAnImuAboveARotatingBase)
{
  const tf2::Transform base_imu(tf2::Quaternion::getIdentity(),tf2::Vector3(0,0,.05));
  const auto result=multimap_nav_simulation::imuTwistAtBase(base_imu,
    tf2::Vector3(.05,0,0),tf2::Vector3(0,1,0));
  EXPECT_NEAR(result.linear.length(),0.,1e-12);
  EXPECT_NEAR(result.angular.y(),1.,1e-12);
}
TEST(NativeTwist, BothVectorsAreRotatedToBaseAxes)
{
  tf2::Quaternion rotation;rotation.setRPY(0,0,M_PI_2);
  const tf2::Transform base_imu(rotation,tf2::Vector3(0,0,0));
  const auto result=multimap_nav_simulation::imuTwistAtBase(base_imu,
    tf2::Vector3(1,0,0),tf2::Vector3(0,1,0));
  EXPECT_NEAR(result.linear.y(),1.,1e-12);
  EXPECT_NEAR(result.angular.x(),-1.,1e-12);
}
