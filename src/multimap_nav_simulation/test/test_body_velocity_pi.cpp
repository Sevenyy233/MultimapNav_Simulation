#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "multimap_nav_simulation/body_velocity_pi.hpp"
using multimap_nav_simulation::VelocityPI;
TEST(BodyVelocityPI, RejectsLinearSlopeBiasWithSensorLag) {
  VelocityPI controller;double speed=0.,measured=0.,last=0.;
  for(int i=0;i<2000;++i) {
    const double command=controller.update(0.,measured,.02);
    speed+=(command+.08-speed)*(-std::expm1(-.02/.1));
    measured+=(speed-measured)*(-std::expm1(-.02/.2));
    if(i>=1900)last=std::max(last,std::abs(speed));
  }
  EXPECT_LT(last,.005);
}
TEST(BodyVelocityPI, TracksAngularRateThroughActuatorDeadzone) {
  VelocityPI controller(1.,2.,.7,1.2);double speed=0.;
  for(double reference:{.25,-.2,0.}) {
    double last=0.;
    for(int i=0;i<1000;++i) {
      const double command=controller.update(reference,speed,.02);
      const double plant=std::copysign(std::max(0.,std::abs(command)-.35),command);
      speed+=(plant-speed)*(-std::expm1(-.02/.12));
      if(i>=900)last=std::max(last,std::abs(speed-reference));
    }
    EXPECT_LT(last,.01);
  }
}
TEST(BodyVelocityPI, BoundsAndResetAreExplicit) {
  VelocityPI controller(1.,2.,.1,.5);
  for(int i=0;i<1000;++i)EXPECT_LE(std::abs(controller.update(2.,0.,.02)),.5);
  controller.reset();EXPECT_DOUBLE_EQ(controller.update(0.,0.,.02),0.);
  EXPECT_THROW(VelocityPI(-1.,2.,.1,.5),std::invalid_argument);
}
TEST(BodyVelocityFeedbackGuard, DivergedFeedbackCannotResumeWithLaterZero) {
  multimap_nav_simulation::VelocityFeedbackGuard guard;
  EXPECT_TRUE(guard.observeLinear(.02,0.,0.));
  EXPECT_FALSE(guard.observeLinear(8.93653716555,0.,0.));
  EXPECT_FALSE(guard.observeLinear(0.,0.,0.));
  EXPECT_EQ(guard.reason(),"linear_feedback_invalid");
}
TEST(BodyVelocityFeedbackGuard, ChecksWholeNormNonfiniteAndFrameFault) {
  multimap_nav_simulation::VelocityFeedbackGuard guard;
  EXPECT_FALSE(guard.observeLinear(2.,2.,2.));
  multimap_nav_simulation::VelocityFeedbackGuard gyro;
  EXPECT_FALSE(gyro.observeAngular(0.,0.,std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(gyro.usable());
  EXPECT_TRUE(gyro.observeAngular(0.,0.,0.));
  EXPECT_FALSE(gyro.observeAngular(0.,4.,0.));
  EXPECT_TRUE(gyro.observeAngular(0.,0.,0.));
  multimap_nav_simulation::VelocityFeedbackGuard frame;
  frame.invalidate("odometry_frame_invalid");EXPECT_FALSE(frame.usable());
  EXPECT_THROW(multimap_nav_simulation::VelocityFeedbackGuard(0.,3.),std::invalid_argument);
}
