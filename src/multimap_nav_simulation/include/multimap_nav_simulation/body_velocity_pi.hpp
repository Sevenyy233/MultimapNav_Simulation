#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace multimap_nav_simulation {
// Implausible odometry/frame faults latch. Angular validity is checked for each
// sample so a transient roll/pitch impulse cannot permanently stop a sequence.
class VelocityFeedbackGuard {
public:
  VelocityFeedbackGuard(double linear=3.,double angular=3.):linear_(linear),angular_(angular) {
    if(!std::isfinite(linear)||!std::isfinite(angular)||linear<=0.||angular<=0.)
      throw std::invalid_argument("Invalid feedback speed bounds");
  }
  bool observeLinear(double x,double y,double z) {return observe(x,y,z,linear_,"linear_feedback_invalid");}
  bool observeAngular(double x,double y,double z) const {return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&std::hypot(x,y,z)<=angular_;}
  void invalidate(const std::string & reason) {if(reason_.empty())reason_=reason;}
  bool usable() const {return reason_.empty();}
  const std::string & reason() const {return reason_;}
private:
  bool observe(double x,double y,double z,double limit,const char * reason) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||std::hypot(x,y,z)>limit)invalidate(reason);
    return usable();
  }
  double linear_,angular_;std::string reason_;
};
class VelocityPI {
public:
  VelocityPI(double kp=1.,double ki=2.,double integral_limit=.3,double command_limit=2.)
  :kp_(kp),ki_(ki),integral_limit_(integral_limit),command_limit_(command_limit) {
    if(!std::isfinite(kp)||!std::isfinite(ki)||!std::isfinite(integral_limit)||!std::isfinite(command_limit)||
       kp<0.||ki<0.||integral_limit<0.||command_limit<=0.)throw std::invalid_argument("Invalid velocity PI bounds");
  }
  double update(double reference,double measured,double dt) {
    if(!std::isfinite(reference)||!std::isfinite(measured)||!std::isfinite(dt)||dt<0.) {reset();return 0.;}
    const double error=reference-measured;
    const double proposed=std::clamp(integral_+ki_*error*dt,-integral_limit_,integral_limit_);
    const double unsaturated=reference+kp_*error+proposed;
    if(!((unsaturated>command_limit_&&error>0.)||(unsaturated<-command_limit_&&error<0.)))integral_=proposed;
    return std::clamp(reference+kp_*error+integral_,-command_limit_,command_limit_);
  }
  void reset(){integral_=0.;}
private:
  double kp_,ki_,integral_limit_,command_limit_,integral_{0.};
};
}
