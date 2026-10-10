#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <algorithm>
#include <functional>
#include <cmath>
#include <vector>

namespace gazebo {
class PathObstacleMotion : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr settings) override {
    model_=std::move(model);
    speed_=settings->HasElement("speed")?settings->Get<double>("speed"):.25;
    delay_=settings->HasElement("start_delay")?settings->Get<double>("start_delay"):4.;
    if(speed_<=0. || !settings->HasElement("waypoints"))return;
    auto list=settings->GetElement("waypoints");
    if(!list->HasElement("point"))return;
    for(auto point=list->GetElement("point");point;point=point->GetNextElement("point"))path_.push_back(point->Get<ignition::math::Vector3d>());
    if(path_.size()<2)return;
    lengths_.push_back(0.);
    for(size_t i=1;i<path_.size();++i)lengths_.push_back(lengths_.back()+(path_[i]-path_[i-1]).Length());
    if(lengths_.back()<.01)return;
    connection_=event::Events::ConnectWorldUpdateBegin(std::bind(&PathObstacleMotion::Update,this,std::placeholders::_1));
  }
 private:
  void Update(const common::UpdateInfo & info) {
    const double elapsed=info.simTime.Double()-delay_;
    if(elapsed<0.)return;
    const double total=lengths_.back(),phase=std::fmod(elapsed*speed_,2.*total);
    const double distance=phase<=total?phase:2.*total-phase;
    auto upper=std::upper_bound(lengths_.begin(),lengths_.end(),distance);
    const size_t i=std::min(path_.size()-1,std::max<size_t>(1,upper-lengths_.begin()));
    const double ratio=(distance-lengths_[i-1])/std::max(1e-8,lengths_[i]-lengths_[i-1]);
    const auto point=path_[i-1]+(path_[i]-path_[i-1])*ratio;
    // The Gazebo entity itself moves, so collision geometry and LiDAR see it.
    // No obstacle pose is injected into any navigation component.
    auto pose=model_->WorldPose();pose.Pos()=point;model_->SetWorldPose(pose);
  }
  physics::ModelPtr model_;event::ConnectionPtr connection_;
  std::vector<ignition::math::Vector3d> path_;std::vector<double> lengths_;
  double speed_=.25,delay_=4.;
};
GZ_REGISTER_MODEL_PLUGIN(PathObstacleMotion)
}  // namespace gazebo
